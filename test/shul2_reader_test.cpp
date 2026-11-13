// Tests for the SHUL/2 READER — diag/shul2_reader.hpp over captures built with the format's
// own encoders (chunk H1 Part A1; the format's goldens are in shul2_format_test.cpp). Every
// test tries to BREAK a rule the reader's header states. What each group targets:
//
//  * TRANSPORT: real transforms of a whole capture (bit 7 stripped, C-string cuts, newline
//    translation both ways), CRC NOT recomputed — the capture status must name each, and not
//    one frame may be delivered from a damaged session.
//  * INTEGRITY: every single-bit flip and an 0xFF XOR at every byte of one Tick frame, in two
//    placements — exactly that frame is lost, everything after it (the End included) arrives.
//    The named tail case (a length bit flipped near the end), a garbage prefix with false sync
//    pairs, truncation at every offset of the last frame.
//  * VERSIONS: an unknown future 1.7 stream read by this 1.0 build (appended bytes ignored and
//    counted, the prefix bit-identical), a 2.0 session refused whole, and this build's capture
//    read by a hypothetical 1.1 reader.
//  * SESSIONS AND SEQUENCE: late joins (and the re-Hello after one), repeats (at a writer's
//    real 100-frame cadence), mismatches, a writer that restarts without an End, stale frames
//    (a stale re-Hello and a stale End included), the 2^31 stale boundary, the 2^32 wrap, and
//    loss figures with their saturation.
//  * LIVE READING: a corrupted length stalls a live reassembler until 14 + 4000 bytes, then the
//    frames inside are recovered; one near the END of a stream holds its frames until the stream
//    ends, and the tail rule then releases them; and in every chunking the live frames equal the
//    whole-capture reader's (the chunking property).
//
// The H1 Part A1 review (2026-09-29) added the cases that name a review mutation (R… / N…): each
// one's mutant survived the first version of this suite.

#include "doctest.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "shul2_test_support.hpp"

using namespace shul2test;

namespace {

/// The bytes of frame i of `c`.
Bytes frameBytesOf(const Capture& c, std::size_t i) {
    return Bytes(c.bytes.begin() + static_cast<std::ptrdiff_t>(c.starts[i]),
                 c.bytes.begin() + static_cast<std::ptrdiff_t>(c.starts[i] + c.frameBytes(i)));
}

/// The capture with frame i removed (its bytes cut out, the rest untouched).
Bytes without(const Capture& c, std::size_t i) {
    Bytes b = c.bytes;
    b.erase(b.begin() + static_cast<std::ptrdiff_t>(c.starts[i]),
            b.begin() + static_cast<std::ptrdiff_t>(c.starts[i] + c.frameBytes(i)));
    return b;
}

std::vector<std::uint32_t> seqsOf(const std::vector<Delivered>& frames) {
    std::vector<std::uint32_t> s;
    for (const auto& f : frames) {
        s.push_back(f.seq);
    }
    return s;
}

/// A payload with `extra` bytes of 0xEE appended (a later minor's appended fields).
Bytes appended(Bytes p, std::size_t extra) {
    p.insert(p.end(), extra, std::byte{0xEE});
    return p;
}

/// A Hello payload at major.minor declaring `row` and `row.hello` bytes long.
Bytes helloAt(std::uint16_t major, std::uint16_t minor, const s2::WidthRow& row) {
    Bytes p = helloPayload();
    putLe(p, 8, major, 2);
    putLe(p, 10, minor, 2);
    putLe(p, 16, row.tick, 2);
    putLe(p, 18, row.summary, 2);
    putLe(p, 20, row.end, 2);
    putLe(p, 22, row.logFixed, 2);
    p.resize(row.hello, std::byte{0});
    return p;
}

constexpr s2::WidthRow kRow10{256, 428, 168, 32, 14};
constexpr s2::WidthRow kRow11{264, 436, 176, 40, 22};
constexpr s2::WidthRow kRows11[] = {kRow10, kRow11};
constexpr s2::ReaderProfile kReader11{1, std::span<const s2::WidthRow>{kRows11}};

}  // namespace

// ── The clean case, and the empty one ───────────────────────────────────────────────

// Would catch: a reader that loses, reorders or re-types frames on a perfect capture, or counts
// damage that did not happen — every counter that names damage must be ZERO here.
TEST_CASE("SHUL/2 reader: a clean session is delivered whole, and every damage counter is 0") {
    const Capture c = standardSession(30);
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    REQUIRE(frames.size() == c.starts.size());
    for (std::size_t i = 0; i < frames.size(); ++i) {
        CHECK(frames[i].seq == c.seqs[i]);
        CHECK(typeByte(frames[i].type) == c.types[i]);
    }
    const auto& n = reader.counters();
    CHECK(reader.status() == s2::CaptureStatus::Ok);
    CHECK(n.bytesSkipped == 0);
    CHECK(n.framesVerified == frames.size());
    CHECK(n.framesDelivered == frames.size());
    CHECK(n.crcFailures == 0);
    CHECK(n.lengthOverruns == 0);
    CHECK(n.sessions == 1);
    CHECK(n.refusedSessions == 0);
    CHECK(n.lateJoins == 0);
    CHECK(n.framesOutsideSession == 0);
    CHECK(n.unknownTypeSkipped == 0);
    CHECK(n.wrongWidthSkipped == 0);
    CHECK(n.extensionBytesIgnored == 0);
    CHECK(n.staleFrames == 0);
    CHECK(n.lostFrames == 0);
    CHECK(n.linkLostFrames == 0);
    CHECK(n.sessionsLinkLossUnknown == 0);
    CHECK(n.endDisagreements == 0);
    CHECK(n.sessionsEnded == 1);
    CHECK_FALSE(n.truncated);
    const auto& s = reader.latestSession();
    CHECK(s.verdict == s2::Verdict::Exact);
    CHECK_FALSE(s.lateJoin);
    CHECK(s.sawEnd);
    CHECK(s.linkLossKnown);
    CHECK(s.linkLostFrames == 0);
    CHECK(s.hello.routineId == "redLeftTall");
    CHECK(s.hello.epochSeconds == 100.25);
    CHECK(s.end.framesOffered == c.seqs.back());
    CHECK(reader.bytesConsumed() == c.bytes.size());

    const Bytes none;
    s2::Shul2Reader empty{none};
    CHECK(readAll(empty).empty());
    CHECK(empty.status() == s2::CaptureStatus::Empty);
    CHECK_FALSE(empty.hasSession());
}

// ── Transport: real transforms, CRC NOT recomputed ──────────────────────────────────

// Would catch: a transport's damage delivered as data, or named wrongly. (1) and (2) leave no
// frame at all (NoFrame). (3) and (4) — newline translation — leave Ticks that held no CR/LF
// intact, but the signature's CR LF guarantees every Hello dies, so no session starts
// (NoSession). The CRC catches the Hello's damage BEFORE the signature is compared, so none of
// these is ever BadSignature.
TEST_CASE("SHUL/2 reader: bit-7 stripping, C-string cuts and newline translation are named") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 50; ++i) {
        if (i % 2 == 0) {
            c.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
        } else {
            DebugRecord quiet;  // mostly zero bytes: likely to hold no 0x0A / 0x0D at all
            quiet.t = units::Time{static_cast<double>(i)};
            c.add(s2::FrameType::Tick, tickPayload(quiet));
        }
        if (i % 10 == 5) {
            c.add(s2::FrameType::Log, logPayload(2, "MOT", "line", 1.0 * i));
        }
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));

    // At least one Tick frame — envelope and CRC included — holds no 0x0A or 0x0D byte, so the
    // newline transforms leave it verifiable. Asserted, not assumed.
    bool someTickClean = false;
    for (std::size_t i = 0; i < c.starts.size(); ++i) {
        if (c.types[i] != typeByte(s2::FrameType::Tick)) {
            continue;
        }
        const Bytes f = frameBytesOf(c, i);
        const bool clean = std::none_of(f.begin(), f.end(), [](std::byte b) {
            return b == std::byte{0x0A} || b == std::byte{0x0D};
        });
        someTickClean = someTickClean || clean;
    }
    REQUIRE(someTickClean);

    Bytes stripped = c.bytes;  // (1) every byte AND 0x7F
    for (auto& b : stripped) {
        b &= std::byte{0x7F};
    }
    Bytes cstrings;  // (2) each frame cut at its first 0x00, one write per frame
    for (std::size_t i = 0; i < c.starts.size(); ++i) {
        const Bytes f = frameBytesOf(c, i);
        const auto nul = std::find(f.begin(), f.end(), std::byte{0x00});
        cstrings.insert(cstrings.end(), f.begin(), nul);
    }
    Bytes lfToCrlf;  // (3) every 0x0A → 0x0D 0x0A
    for (const std::byte b : c.bytes) {
        if (b == std::byte{0x0A}) {
            lfToCrlf.push_back(std::byte{0x0D});
        }
        lfToCrlf.push_back(b);
    }
    Bytes crlfToLf;  // (4) every 0x0D 0x0A → 0x0A
    for (std::size_t i = 0; i < c.bytes.size(); ++i) {
        if (c.bytes[i] == std::byte{0x0D} && i + 1 < c.bytes.size() && c.bytes[i + 1] == std::byte{0x0A}) {
            continue;
        }
        crlfToLf.push_back(c.bytes[i]);
    }

    struct Case {
        const char* name;
        const Bytes* bytes;
        s2::CaptureStatus want;
    };
    const Case cases[] = {{"AND 0x7F", &stripped, s2::CaptureStatus::NoFrame},
                          {"C-string cut", &cstrings, s2::CaptureStatus::NoFrame},
                          {"LF -> CR LF", &lfToCrlf, s2::CaptureStatus::NoSession},
                          {"CR LF -> LF", &crlfToLf, s2::CaptureStatus::NoSession}};
    for (const Case& k : cases) {
        CAPTURE(k.name);
        REQUIRE(*k.bytes != c.bytes);  // the transform did something
        s2::Shul2Reader reader{*k.bytes};
        CHECK(readAll(reader).empty());
        CHECK(reader.counters().sessions == 0);
        CHECK(reader.status() == k.want);
        if (k.want == s2::CaptureStatus::NoSession) {
            CHECK(reader.counters().framesOutsideSession >= 1);
        }
    }
}

// ── Forward compatibility (CRC recomputed: these are a later writer's bytes) ─────────

// Would catch: a reader that judges what 1.0 reserves — envelope flags (mutation 22), a Hello's
// flags or reserved bytes (mutation 23), a Log's reserved byte or flag bits 2-7, the Tick's
// flag bits 3-7 (TANK1 gave bit 2 a meaning, so 3-7 are what is left) — or that delivers by
// role instead of by type.
TEST_CASE("SHUL/2 reader: flags, reserved bytes and roles a later minor may use are ignored") {
    {  // envelope flags 0xFF on every frame
        Capture c;
        c.add(s2::FrameType::Hello, helloPayload(), 0xFF);
        c.add(s2::FrameType::Tick, tickPayload(record(0.0)), 0xFF);
        c.add(s2::FrameType::End, endPayloadAt(2), 0xFF);
        s2::Shul2Reader reader{c.bytes};
        const auto frames = readAll(reader);
        REQUIRE(frames.size() == 3);
        for (const auto& f : frames) {
            CHECK(f.flags == 0xFF);  // reported, never judged
        }
        CHECK(reader.status() == s2::CaptureStatus::Ok);
    }
    {  // Hello flags 0xFFFF and every reserved byte 0xFF
        Bytes h = helloPayload();
        h[13] = std::byte{0xFF};
        putLe(h, 14, 0xFFFF, 2);
        putLe(h, 34, 0xFFFF, 2);
        for (std::size_t i = 244; i < 256; ++i) {
            h[i] = std::byte{0xFF};
        }
        Capture c;
        c.add(s2::FrameType::Hello, h);
        c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
        s2::Shul2Reader reader{c.bytes};
        CHECK(readAll(reader).size() == 2);
        CHECK(reader.latestSession().verdict == s2::Verdict::Exact);
        CHECK(reader.latestSession().hello.flags == 0xFFFF);
    }
    {  // Log reserved byte 0xFF and flag bits 2-7 set
        Bytes l = logPayload(3, "ODO", "reserved bits", 5.0);
        l[1] = std::byte{0xFE};  // bits 1-7 (bit 1 is "message truncated"; 2-7 are reserved)
        l[3] = std::byte{0xFF};
        Capture c;
        c.add(s2::FrameType::Hello, helloPayload());
        c.add(s2::FrameType::Log, l);
        s2::Shul2Reader reader{c.bytes};
        const auto frames = readAll(reader);
        REQUIRE(frames.size() == 2);
        s2::LogLine line;
        REQUIRE(s2::decodeLog(frames[1].payload, line));
        CHECK(line.level == 3);
        CHECK(line.tag == "ODO");
        CHECK(line.message == "reserved bits");
        CHECK((line.flags & s2::kLogMessageTruncated) != 0);
        CHECK((line.flags & s2::kLogTagTruncated) == 0);
    }
    {  // Tick flag bits 3-7 set: bits 0-2 unchanged
        for (const std::uint8_t low : {std::uint8_t{0x05}, std::uint8_t{0x02}}) {
            CAPTURE(static_cast<int>(low));
            Bytes t = tickPayload(record(0.0));
            t[123] = static_cast<std::byte>(0xF8U | low);
            Capture c;
            c.add(s2::FrameType::Hello, helloPayload());
            c.add(s2::FrameType::Tick, t);
            s2::Shul2Reader reader{c.bytes};
            const auto frames = readAll(reader);
            REQUIRE(frames.size() == 2);
            DebugRecord out;
            bool corrupt = false;
            REQUIRE(s2::decodeTick(frames[1].payload, out, corrupt));
            CHECK_FALSE(corrupt);
            CHECK(out.clampedThisTick == ((low & 1U) != 0));
            CHECK(out.strafeFallbackActive == ((low & 2U) != 0));
            CHECK(out.tankSteerActive == ((low & 4U) != 0));
        }
    }
    for (const std::uint8_t role : {std::uint8_t{1}, std::uint8_t{7}}) {  // roles 1 and 7
        CAPTURE(static_cast<int>(role));
        Capture c;
        c.add(s2::FrameType::Hello, helloPayload(standardInfo(), 1.0, role));
        c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
        c.add(s2::FrameType::Log, logPayload(2, "X", "y", 1.0));
        c.add(s2::FrameType::End, endPayloadAt(3));
        s2::Shul2Reader reader{c.bytes};
        const auto frames = readAll(reader);
        REQUIRE(frames.size() == 4);
        CHECK(frames[1].type == s2::FrameType::Tick);
        CHECK(frames[2].type == s2::FrameType::Log);
        CHECK(reader.latestSession().hello.role == role);
        CHECK(reader.latestSession().verdict == s2::Verdict::Exact);
    }
}

// ── Framing and integrity ───────────────────────────────────────────────────────────

// Would catch: an unknown type delivered, or treated as a sequence gap, or stopping the stream.
TEST_CASE("SHUL/2 reader: types 0, 60, 64 and 200 are skipped by length, counted, and the stream continues") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    for (const std::uint8_t t : {std::uint8_t{0}, std::uint8_t{60}, std::uint8_t{64}, std::uint8_t{200}}) {
        c.addAt(t, c.nextSeq, Bytes(17, std::byte{0x5A}));
        c.add(s2::FrameType::Tick, tickPayload(record(1.0)));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(frames.size() == 1 + 5 + 1);  // Hello, five Ticks, End
    CHECK(reader.counters().unknownTypeSkipped == 4);
    CHECK(reader.counters().lostFrames == 0);  // their sequence numbers were accounted for
    CHECK(reader.latestSession().sawEnd);
    CHECK(reader.counters().bytesSkipped == 0);  // they VERIFIED: skipped by type, not as bytes
}

// Would catch: a known type at a width its session did not declare, delivered (a tick one
// byte short would be decoded as confident nonsense).
TEST_CASE("SHUL/2 reader: a known type at the wrong width is counted, never delivered") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, Bytes(427));
    c.add(s2::FrameType::Tick, Bytes(429));
    c.add(s2::FrameType::Summary, Bytes(167));
    c.add(s2::FrameType::End, Bytes(33));
    Bytes badLog = logPayload(2, "T", "m", 1.0);
    badLog.push_back(std::byte{0});  // non-text 15, but the Hello declared 14
    c.add(s2::FrameType::Log, badLog);
    c.add(s2::FrameType::Log, Bytes(13));  // shorter than the fixed part
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    REQUIRE(frames.size() == 2);
    CHECK(frames[1].seq == 7);
    CHECK(reader.counters().wrongWidthSkipped == 6);
    CHECK_FALSE(reader.latestSession().sawEnd);  // the mis-sized End closed nothing
}

namespace {

/// The flip loop's verdict on one damaged capture: exactly the target lost, all else delivered.
bool exactlyTargetLost(const Capture& c, std::size_t target, const Bytes& damaged, std::string& why) {
    s2::Shul2Reader reader{damaged};
    const auto frames = readAll(reader);
    std::vector<std::uint32_t> want = c.seqs;
    want.erase(want.begin() + static_cast<std::ptrdiff_t>(target));
    const auto& n = reader.counters();
    const auto& s = reader.latestSession();
    why = "delivered=" + std::to_string(frames.size()) + " lost=" + std::to_string(n.lostFrames)
          + " skipped=" + std::to_string(n.bytesSkipped) + " truncated=" + std::to_string(n.truncated)
          + " sawEnd=" + std::to_string(s.sawEnd);
    return seqsOf(frames) == want && n.lostFrames == 1 && s.lostFrames == 1 && !n.truncated
           && s.sawEnd && s.linkLossKnown && s.linkLostFrames == 1
           && n.bytesSkipped == c.frameBytes(target)  // exactly the damaged frame's bytes
           && reader.status() == s2::CaptureStatus::Ok;
}

}  // namespace

// Would catch: a reader that skips the CRC (mutation 2), resyncs at the claimed length
// (mutation 3), stops at a tail NeedMore (20), or marks truncation after a later frame verified
// (21). For ONE Tick frame, every single-bit flip and an 0xFF XOR at every one of its 442 bytes
// (3,978 damaged captures per placement): exactly that frame is lost; every later frame, the End
// included, is delivered; lostFrames == 1; truncated is false; and the skipped bytes are exactly
// the damaged frame's. Placement 1 has more than 4110 bytes after the frame (a grown length
// fails the CRC); placement 2 makes it the third frame before the End (a grown length runs past
// the end of the capture — the tail rule).
TEST_CASE("SHUL/2 reader: the flip loop — every bit of one Tick frame, in two placements") {
    Capture mid;
    mid.add(s2::FrameType::Hello, helloPayload());
    mid.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    mid.add(s2::FrameType::Tick, tickPayload(record(1.0)));
    mid.add(s2::FrameType::Tick, tickPayload(record(2.0)));  // the target: frame 3
    for (int i = 3; i < 13; ++i) {
        mid.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
    }
    mid.add(s2::FrameType::End, endPayloadAt(mid.nextSeq));
    REQUIRE(mid.bytes.size() - mid.starts[3] - 442 > 4110);

    Capture tail;
    tail.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 4; ++i) {
        tail.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
    }  // frame 4 is the target: the third frame before the End
    tail.add(s2::FrameType::Tick, tickPayload(record(4.0)));
    tail.add(s2::FrameType::Tick, tickPayload(record(5.0)));
    tail.add(s2::FrameType::End, endPayloadAt(tail.nextSeq));

    struct Placement {
        const char* name;
        const Capture* c;
        std::size_t target;
    };
    const Placement placements[] = {{"mid-capture", &mid, 3}, {"third before the End", &tail, 4}};
    const std::uint8_t masks[9] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF};
    for (const Placement& p : placements) {
        REQUIRE(p.c->frameBytes(p.target) == 442);
        int failures = 0;
        for (std::size_t pos = 0; pos < 442; ++pos) {
            for (const std::uint8_t mask : masks) {
                Bytes damaged = p.c->bytes;
                damaged[p.c->starts[p.target] + pos] ^= static_cast<std::byte>(mask);
                std::string why;
                if (!exactlyTargetLost(*p.c, p.target, damaged, why) && failures < 5) {
                    ++failures;
                    CAPTURE(p.name);
                    CAPTURE(pos);
                    CAPTURE(static_cast<int>(mask));
                    CAPTURE(why);
                    FAIL_CHECK("a single-byte corruption cost more (or less) than its own frame");
                }
            }
        }
        CAPTURE(p.name);
        CHECK(failures == 0);
    }
}

// Would catch: the blackbox's stop-and-mark-truncated rule applied to a checked stream
// (mutation 20) and `truncated` set although a later frame verified (21). The named case: the
// third frame before the End with its length 428 = 0x01AC turned into 2476 = 0x09AC by one bit
// flip — it claims more bytes than the capture has left.
TEST_CASE("SHUL/2 reader: the tail case — a length of 2476 near the end is an overrun, not a cut") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 5; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    const std::size_t target = 3;  // ticks at frames 1-5, End at 6: frame 3 is third before the End
    Bytes b = c.bytes;
    REQUIRE(le16(b, c.starts[target] + 8) == 428);
    putLe(b, c.starts[target] + 8, 2476, 2);
    REQUIRE(c.bytes.size() - c.starts[target] < 14 + 2476);  // it really runs past the end

    s2::Shul2Reader reader{b};
    const auto frames = readAll(reader);
    CHECK(seqsOf(frames) == std::vector<std::uint32_t>{0, 1, 2, 4, 5, 6});
    CHECK(reader.counters().lengthOverruns >= 1);
    CHECK_FALSE(reader.counters().truncated);
    CHECK(reader.latestSession().sawEnd);
    CHECK(reader.counters().lostFrames == 1);
}

// Would catch: a garbage prefix that swallows the first real frame, or is only partly counted.
// False sync pairs inside it: one with a plausible length (its CRC fails), one above 4096, and
// one whose length runs past the end of the capture.
TEST_CASE("SHUL/2 reader: a garbage prefix with false sync pairs is skipped up to the Hello") {
    const Bytes prefix = {
        std::byte{0x00}, std::byte{0x53}, std::byte{0xFE}, std::byte{0x53}, std::byte{0x02},
        std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x10}, std::byte{0x00},  // a false frame of 16 bytes: its CRC fails
        std::byte{0xFE}, std::byte{0x53}, std::byte{0x02}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0xFF}, std::byte{0xFF},  // 65535
        std::byte{0xFE}, std::byte{0x53}, std::byte{0x03}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x10},  // 4096: past the end
        std::byte{0xFE}, std::byte{0xFE}, std::byte{0x7F}, std::byte{0x53}};
    Capture c;
    c.raw(prefix);
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    c.add(s2::FrameType::End, endPayloadAt(2));
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(seqsOf(frames) == std::vector<std::uint32_t>{0, 1, 2});
    CHECK(reader.counters().bytesSkipped == prefix.size());
    CHECK(reader.counters().crcFailures >= 1);
    CHECK_FALSE(reader.counters().truncated);
}

// Would catch: the 4096 bound applied as `>=` in the reader's path (review mutation N3/R2) — the
// largest frame a writer may legally send, a Log whose tag and message fill the payload, skipped
// as LengthTooLarge by every reader. The tag is also the longest a u8 can declare, 255.
TEST_CASE("SHUL/2 reader: a Log of exactly 4096 payload bytes is delivered") {
    const std::string tag(255, 'T');
    const std::string message(4096 - 14 - 255, 'm');
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Log, logPayload(2, tag, message, 3.5));
    c.add(s2::FrameType::End, endPayloadAt(2));
    REQUIRE(c.frameBytes(1) == s2::kMaxFrameBytes);
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    REQUIRE(seqsOf(frames) == std::vector<std::uint32_t>{0, 1, 2});
    CHECK(reader.counters().bytesSkipped == 0);
    CHECK(reader.counters().wrongWidthSkipped == 0);
    s2::LogLine line;
    REQUIRE(s2::decodeLog(frames[1].payload, line));
    CHECK(line.tag == tag);
    CHECK(line.message == message);
    CHECK(line.nonTextBytes == 14);
}

// Would catch: a cut capture that loses frames BEFORE the cut, a cut not reported, the wrong
// truncatedAt, a clean end reported as truncated, and an End claimed that never arrived. The
// last frame is the End, cut at every offset 1 … 45; offset 0 is a clean frame boundary.
TEST_CASE("SHUL/2 reader: a capture cut at every offset of its last frame") {
    const Capture c = standardSession(6);
    const std::size_t last = c.starts.size() - 1;
    const std::size_t start = c.starts[last];
    REQUIRE(c.frameBytes(last) == 14 + 32);
    for (std::size_t cut = 0; cut < c.frameBytes(last); ++cut) {
        CAPTURE(cut);
        const Bytes b(c.bytes.begin(), c.bytes.begin() + static_cast<std::ptrdiff_t>(start + cut));
        s2::Shul2Reader reader{b};
        const auto frames = readAll(reader);
        CHECK(frames.size() == last);  // every earlier frame
        CHECK_FALSE(reader.latestSession().sawEnd);
        if (cut == 0) {
            CHECK_FALSE(reader.counters().truncated);
        } else {
            CHECK(reader.counters().truncated);
            CHECK(reader.counters().truncatedAt == start);
        }
        CHECK(reader.counters().sessionsLinkLossUnknown == 1);  // no End: link loss unknowable
    }
}

// Would catch: truncatedAt taken from the LATEST tail candidate instead of the EARLIEST (review
// mutations N2/R4) — it would point into the middle of the cut frame. The End in the case above
// holds no false candidate, so earliest and latest coincide there. Here the cut frame is a Log
// whose message carries a false frame header (FE 53, then a length of 4000 that runs past any
// cut), so every cut past it leaves at least TWO candidates that run off the end: the Log's own
// start and the false one inside it. Asserted, not assumed.
TEST_CASE("SHUL/2 reader: truncatedAt is the cut frame's start, even with a false frame inside it") {
    Capture c = standardSession(3, false);
    std::string message = "abc";
    const unsigned char falseHeader[10] = {0xFE, 0x53, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x0F};
    for (const unsigned char b : falseHeader) {
        message.push_back(static_cast<char>(b));
    }
    message += "tail text";
    c.add(s2::FrameType::Log, logPayload(2, "RUN", message, 9.0));
    const std::size_t last = c.starts.size() - 1;
    const std::size_t start = c.starts[last];
    const std::size_t falseAt = start + 10 + 14 + 3 + 3;  // envelope, Log fixed bytes, "RUN", "abc"
    REQUIRE(c.bytes[falseAt] == std::byte{0xFE});
    REQUIRE(c.bytes[falseAt + 1] == std::byte{0x53});
    for (std::size_t cut = 1; cut < c.frameBytes(last); ++cut) {
        CAPTURE(cut);
        const Bytes b(c.bytes.begin(), c.bytes.begin() + static_cast<std::ptrdiff_t>(start + cut));
        if (start + cut > falseAt) {  // the false candidate is in the capture: it runs off the end too
            REQUIRE(s2::scanFrame(std::span<const std::byte>{b}.subspan(falseAt)).kind == s2::ScanKind::NeedMore);
        }
        s2::Shul2Reader reader{b};
        const auto frames = readAll(reader);
        CHECK(frames.size() == last);
        CHECK(reader.counters().truncated);
        CHECK(reader.counters().truncatedAt == start);
        CHECK(reader.counters().lengthOverruns == 0);  // nothing verified after either candidate
    }
}

// ── Sequence and link loss ──────────────────────────────────────────────────────────

// Would catch: a gap not counted, and link loss confused with the writer's own refusals.
TEST_CASE("SHUL/2 reader: a frame cut out of a session is one lost frame and one link-lost frame") {
    const Capture c = standardSession(10);
    const Bytes cut = without(c, 5);  // named: the reader BORROWS its span, never a temporary's
    s2::Shul2Reader reader{cut};
    (void)readAll(reader);
    CHECK(reader.counters().lostFrames == 1);
    CHECK(reader.counters().linkLostFrames == 1);
    CHECK(reader.latestSession().linkLossKnown);
    CHECK(reader.latestSession().linkLostFrames == 1);
    CHECK(reader.counters().endDisagreements == 0);
}

// Would catch: a device refusal (the End's framesFailed) counted against the LINK.
TEST_CASE("SHUL/2 reader: a gap the End accounts for as a device refusal is not link loss") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (std::uint32_t s = 1; s <= 9; ++s) {
        if (s == 5) {
            continue;  // the device refused it: the writer counted it, and it shows as a gap
        }
        c.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(1.0 * s)));
    }
    c.addAt(typeByte(s2::FrameType::End), 10, endPayloadAt(10, 1));
    s2::Shul2Reader reader{c.bytes};
    (void)readAll(reader);
    CHECK(reader.counters().lostFrames == 1);
    CHECK(reader.latestSession().linkLossKnown);
    CHECK(reader.latestSession().linkLostFrames == 0);
    CHECK(reader.counters().linkLostFrames == 0);
    CHECK(reader.counters().endDisagreements == 0);
}

// Would catch: link loss computed for a late-joined session (mutation 24: it would subtract
// refusals from gaps it never saw, and without saturation wrap to ~4 billion); and a late join's
// End exempted from the framesOffered check (review mutation N12) — that check needs nothing
// from before the join: the End's own sequence number is right there.
TEST_CASE("SHUL/2 reader: a late join's link loss is unknown, and nothing wraps") {
    const auto lateJoined = [](const Bytes& end) {
        Capture c;
        for (std::uint32_t s = 1; s <= 3; ++s) {  // the stream was already running
            c.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(1.0 * s)));
        }
        c.addAt(typeByte(s2::FrameType::Hello), 4, helloPayload());  // a re-Hello: the join point
        for (std::uint32_t s = 5; s <= 8; ++s) {
            c.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(1.0 * s)));
        }
        c.addAt(typeByte(s2::FrameType::End), 9, end);
        return c;
    };
    const Capture c = lateJoined(endPayloadAt(9, 5));  // 5 refused, 0 gaps seen
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(seqsOf(frames) == std::vector<std::uint32_t>{4, 5, 6, 7, 8, 9});
    CHECK(reader.counters().lateJoins == 1);
    CHECK(reader.counters().framesOutsideSession == 3);
    CHECK(reader.latestSession().lateJoin);
    CHECK_FALSE(reader.latestSession().linkLossKnown);
    CHECK(reader.latestSession().linkLostFrames == 0);
    CHECK(reader.counters().linkLostFrames == 0);
    CHECK(reader.counters().sessionsLinkLossUnknown == 1);
    CHECK(reader.counters().endDisagreements == 0);  // gaps before the join were never seen
    CHECK(reader.status() == s2::CaptureStatus::Ok);

    const Capture wrong = lateJoined(endPayloadAt(99));  // framesOffered 99 on the End at seq 9
    s2::Shul2Reader r2{wrong.bytes};
    CHECK(readAll(r2).size() == 6);
    CHECK(r2.counters().endDisagreements == 1);
    CHECK_FALSE(r2.latestSession().linkLossKnown);
}

// Would catch: link loss claimed for a session whose End never came.
TEST_CASE("SHUL/2 reader: a session with no End has unknown link loss") {
    const Capture c = standardSession(5, false);
    s2::Shul2Reader reader{c.bytes};
    (void)readAll(reader);
    CHECK_FALSE(reader.latestSession().sawEnd);
    CHECK_FALSE(reader.latestSession().linkLossKnown);
    CHECK(reader.counters().sessionsLinkLossUnknown == 1);
    CHECK_FALSE(reader.counters().truncated);  // it ended on a frame boundary
}

// Would catch: an End that disagrees with what the reader saw passing silently, and link loss
// that wraps below zero instead of saturating.
TEST_CASE("SHUL/2 reader: an End that disagrees is counted, and link loss saturates at 0") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 5; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq, 2));  // claims 2 refusals; 0 gaps seen
    s2::Shul2Reader reader{c.bytes};
    (void)readAll(reader);
    CHECK(reader.counters().endDisagreements == 1);
    CHECK(reader.latestSession().linkLossKnown);
    CHECK(reader.latestSession().linkLostFrames == 0);
    CHECK(reader.counters().linkLostFrames == 0);

    Capture d;  // an End whose framesOffered is not its own sequence number
    d.add(s2::FrameType::Hello, helloPayload());
    d.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    d.add(s2::FrameType::End, endPayloadAt(99));
    s2::Shul2Reader r2{d.bytes};
    (void)readAll(r2);
    CHECK(r2.counters().endDisagreements == 1);
}

// Would catch: a loss figure that WRAPS instead of saturating — the per-session and total lost
// frames (review mutation N5) and the link-loss total (N21), which the reader's header promises
// saturate. Reachable with a handful of frames: one gap may be as large as 2^31 − 2 frames, so
// three of them overflow a u32, and a second session's single lost frame overflows the total.
TEST_CASE("SHUL/2 reader: lost and link-lost frames saturate at 0xFFFFFFFF, never wrap") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());  // expected 1 after it
    std::uint32_t seq = 0;
    for (int i = 0; i < 3; ++i) {
        seq += 0x7FFFFFFFU;  // each Tick lands 0x7FFFFFFE past the expected seq: the largest gap
        c.addAt(typeByte(s2::FrameType::Tick), seq, tickPayload(record(1.0 * i)));
    }
    c.addAt(typeByte(s2::FrameType::End), seq + 1U, endPayloadAt(seq + 1U));
    {
        s2::Shul2Reader reader{c.bytes};
        CHECK(readAll(reader).size() == 5);
        CHECK(reader.counters().staleFrames == 0);
        CHECK(reader.latestSession().lostFrames == 0xFFFFFFFFU);  // 3 × 0x7FFFFFFE, saturated
        CHECK(reader.counters().lostFrames == 0xFFFFFFFFU);
        CHECK(reader.latestSession().linkLossKnown);
        CHECK(reader.latestSession().linkLostFrames == 0xFFFFFFFFU);
        CHECK(reader.counters().linkLostFrames == 0xFFFFFFFFU);
    }
    // A second session that loses one more frame: the totals stay saturated instead of wrapping.
    c.addAt(typeByte(s2::FrameType::Hello), 0, helloPayload());
    c.addAt(typeByte(s2::FrameType::Tick), 1, tickPayload(record(1.0)));
    c.addAt(typeByte(s2::FrameType::Tick), 3, tickPayload(record(3.0)));  // seq 2 lost
    c.addAt(typeByte(s2::FrameType::End), 4, endPayloadAt(4));
    s2::Shul2Reader both{c.bytes};
    CHECK(readAll(both).size() == 9);
    CHECK(both.counters().sessions == 2);
    CHECK(both.latestSession().linkLostFrames == 1);
    CHECK(both.counters().lostFrames == 0xFFFFFFFFU);
    CHECK(both.counters().linkLostFrames == 0xFFFFFFFFU);
}

// Would catch: the sequence gap computed without the modulo-2^32 wrap (mutation 5) — a wrap
// must be in order and start nothing, and a frame lost ACROSS the wrap is one lost frame.
TEST_CASE("SHUL/2 reader: the 2^32 wrap is in order, starts nothing, and a loss across it counts once") {
    Capture c;
    c.addAt(typeByte(s2::FrameType::Hello), 0xFFFFFFFDU, helloPayload());  // a late join near the top
    for (const std::uint32_t s : {0xFFFFFFFEU, 0xFFFFFFFFU, 0U, 1U}) {
        c.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(0.5)));
    }
    c.addAt(typeByte(s2::FrameType::End), 2, endPayloadAt(2));
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(frames.size() == 6);
    CHECK(reader.counters().sessions == 1);  // the Tick at seq 0 is not a Hello: it starts nothing
    CHECK(reader.counters().lostFrames == 0);
    CHECK(reader.counters().staleFrames == 0);

    Capture lossy;  // 0xFFFFFFFF is lost
    lossy.addAt(typeByte(s2::FrameType::Hello), 0xFFFFFFFDU, helloPayload());
    for (const std::uint32_t s : {0xFFFFFFFEU, 0U, 1U}) {
        lossy.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(0.5)));
    }
    s2::Shul2Reader r2{lossy.bytes};
    CHECK(seqsOf(readAll(r2)) == std::vector<std::uint32_t>{0xFFFFFFFDU, 0xFFFFFFFEU, 0U, 1U});
    CHECK(r2.counters().lostFrames == 1);
    CHECK(r2.counters().staleFrames == 0);
}

// Would catch: a duplicate or out-of-date frame delivered (mutation 19); a stale re-Hello (a
// duplicate of an earlier one, or one the link delivered late) delivered or counted as a repeat;
// a stale END that still closes its session (review mutation N20 — every frame after it would
// fall outside the session); and the stale boundary moved off 2^31 by one either way (review
// mutations N10/R3: d = 2^31 exactly is stale, d = 2^31 − 1 is that many frames lost).
TEST_CASE("SHUL/2 reader: a duplicate or older frame is stale — skipped and counted") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (const std::uint32_t s : {1U, 2U, 2U, 3U, 1U, 4U}) {
        c.addAt(typeByte(s2::FrameType::Tick), s, tickPayload(record(1.0 * s)));
    }
    s2::Shul2Reader reader{c.bytes};
    CHECK(seqsOf(readAll(reader)) == std::vector<std::uint32_t>{0, 1, 2, 3, 4});
    CHECK(reader.counters().staleFrames == 2);
    CHECK(reader.counters().lostFrames == 0);

    Capture h;  // an identical re-Hello, stale: seq 3 after seq 5
    h.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 5; ++i) {
        h.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    h.addAt(typeByte(s2::FrameType::Hello), 3, helloPayload());
    h.addAt(typeByte(s2::FrameType::Tick), 6, tickPayload(record(6.0)));
    h.addAt(typeByte(s2::FrameType::End), 7, endPayloadAt(7));
    s2::Shul2Reader staleHello{h.bytes};
    CHECK(seqsOf(readAll(staleHello)) == std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5, 6, 7});
    CHECK(staleHello.counters().staleFrames == 1);
    CHECK(staleHello.counters().helloRepeats == 0);  // stale first: never judged as a repeat
    CHECK(staleHello.counters().sessions == 1);
    CHECK(staleHello.counters().lostFrames == 0);
    CHECK(staleHello.latestSession().linkLossKnown);

    Capture e;  // a stale End (seq 2, after seq 3): skipped — the session stays open
    e.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 3; ++i) {
        e.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    e.addAt(typeByte(s2::FrameType::End), 2, endPayloadAt(2));
    e.addAt(typeByte(s2::FrameType::Tick), 4, tickPayload(record(4.0)));
    e.addAt(typeByte(s2::FrameType::End), 5, endPayloadAt(5));
    s2::Shul2Reader staleEnd{e.bytes};
    CHECK(seqsOf(readAll(staleEnd)) == std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5});
    CHECK(staleEnd.counters().staleFrames == 1);
    CHECK(staleEnd.counters().sessionsEnded == 1);
    CHECK(staleEnd.counters().framesOutsideSession == 0);
    CHECK(staleEnd.counters().endDisagreements == 0);
    CHECK(staleEnd.latestSession().end.framesOffered == 5);

    Capture b;  // the boundary: after the Hello, expected = 1
    b.add(s2::FrameType::Hello, helloPayload());
    b.addAt(typeByte(s2::FrameType::Tick), 1U + 0x80000000U, tickPayload(record(1.0)));  // d = 2^31: stale
    b.addAt(typeByte(s2::FrameType::Tick), 1U + 0x7FFFFFFFU, tickPayload(record(2.0)));  // d = 2^31 − 1: lost
    s2::Shul2Reader boundary{b.bytes};
    CHECK(seqsOf(readAll(boundary)) == std::vector<std::uint32_t>{0, 0x80000000U});
    CHECK(boundary.counters().staleFrames == 1);
    CHECK(boundary.counters().lostFrames == 0x7FFFFFFFU);
}

// Would catch: frames before the first Hello delivered with no session to judge them by, a late
// join that is not recognised as one, and — review mutation N28 — a late-joined session that
// does not keep ITS Hello as the one later re-Hellos must repeat: the writer's next periodic
// re-Hello would then look like a different writer and split the session in two. That is the
// normal case for a live attach mid-run (H2), which is what this wire is for.
TEST_CASE("SHUL/2 reader: a capture starting at a random offset delivers from the first Hello") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 100; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    c.add(s2::FrameType::Hello, helloPayload());  // the writer's periodic re-Hello, at seq 101
    for (int i = 0; i < 100; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    c.add(s2::FrameType::Hello, helloPayload());  // the next one, at seq 202
    for (int i = 0; i < 50; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    REQUIRE(c.seqs[202] == 202);
    REQUIRE(c.types[202] == typeByte(s2::FrameType::Hello));
    std::mt19937 rng{20260929U};
    for (int trial = 0; trial < 8; ++trial) {
        const std::size_t cut = std::uniform_int_distribution<std::size_t>{1, c.starts[101] - 1}(rng);
        CAPTURE(cut);
        const Bytes b(c.bytes.begin() + static_cast<std::ptrdiff_t>(cut), c.bytes.end());
        std::uint32_t before = 0;  // whole frames between the cut and the re-Hello
        for (std::size_t i = 0; i < 101; ++i) {
            before += c.starts[i] >= cut ? 1U : 0U;
        }
        s2::Shul2Reader reader{b};
        const auto frames = readAll(reader);
        REQUIRE_FALSE(frames.empty());
        CHECK(frames.front().seq == 101);
        CHECK(frames.front().type == s2::FrameType::Hello);
        CHECK(frames.size() == 1 + 100 + 1 + 50 + 1);
        CHECK(reader.counters().framesOutsideSession == before);
        CHECK(reader.counters().lateJoins == 1);
        CHECK(reader.counters().sessions == 1);
        CHECK(reader.counters().helloRepeats == 1);  // seq 202 repeats the JOINED Hello
        CHECK(reader.counters().helloMismatches == 0);
        CHECK(reader.latestSession().lateJoin);
        CHECK(reader.latestSession().sawEnd);
        CHECK(reader.counters().lostFrames == 0);
        CHECK(reader.counters().staleFrames == 0);
        CHECK(reader.counters().sessionsLinkLossUnknown == 1);  // joined late: unknowable
    }
}

// Would catch: a seq-0 Hello inside a session that never ENDED — a brain reset, a brownout or a
// program restart mid-stream — judged as a repeat or a mismatch of the open session (review
// mutation N26). Identical Hello bytes are realistic here (the same SessionInfo, and a clock that
// restarted gives the same epoch), and as a "repeat" every frame of the restarted session would
// fail the sequence rule as stale: the whole session lost, silently. Both an identical and a
// differing restart Hello must start a NEW session read from its seq-0 Hello.
TEST_CASE("SHUL/2 reader: a writer that restarts without an End starts a new session at its seq-0 Hello") {
    for (const bool identical : {true, false}) {
        CAPTURE(identical);
        Capture c;
        c.add(s2::FrameType::Hello, helloPayload(standardInfo(), 100.25));
        for (int i = 0; i < 5; ++i) {
            c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
        }
        // No End: the writer restarted, and its sequence starts again at 0.
        c.addAt(typeByte(s2::FrameType::Hello), 0, helloPayload(standardInfo(), identical ? 100.25 : 7.5));
        for (int i = 0; i < 3; ++i) {
            c.add(s2::FrameType::Tick, tickPayload(record(10.0 + i)));
        }
        c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));  // seq 4
        s2::Shul2Reader reader{c.bytes};
        const auto frames = readAll(reader);
        CHECK(seqsOf(frames) == std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4});
        const auto& n = reader.counters();
        CHECK(n.sessions == 2);
        CHECK(n.staleFrames == 0);
        CHECK(n.lostFrames == 0);
        CHECK(n.helloRepeats == 0);
        CHECK(n.helloMismatches == 0);
        CHECK(n.lateJoins == 0);
        CHECK(n.sessionsEnded == 1);
        CHECK(n.sessionsLinkLossUnknown == 1);  // the first session: it never sent its End
        const auto& s = reader.latestSession();
        CHECK_FALSE(s.lateJoin);
        CHECK(s.helloSeq == 0);
        CHECK(s.sawEnd);
        CHECK(s.linkLossKnown);
        CHECK(s.linkLostFrames == 0);
        CHECK(s.hello.epochSeconds == (identical ? 100.25 : 7.5));
    }
}

// Would catch: a repeated Hello that starts a new session (mutation 16); a repeat that skips the
// sequence rule (review mutations R5/N1 — its seq is then never accounted, so a PERFECT capture
// reports a lost frame, and a link-lost frame, for every re-Hello); a DIFFERENT Hello at seq > 0
// treated as a repeat; and a mismatch counted as a late join as well (review mutation N7 — the
// lateJoins counter counts only Hellos at seq > 0 with no session open).
TEST_CASE("SHUL/2 reader: an identical re-Hello is a repeat; a different one is a new session") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    c.add(s2::FrameType::Hello, helloPayload());  // identical, seq 2
    c.add(s2::FrameType::Tick, tickPayload(record(1.0)));
    {
        Capture ended = c;
        ended.add(s2::FrameType::End, endPayloadAt(ended.nextSeq));  // seq 4
        s2::Shul2Reader repeat{ended.bytes};
        CHECK(seqsOf(readAll(repeat)) == std::vector<std::uint32_t>{0, 1, 2, 3, 4});  // delivered like any frame
        const auto& n = repeat.counters();
        CHECK(n.sessions == 1);
        CHECK(n.helloRepeats == 1);
        CHECK(n.helloMismatches == 0);
        CHECK(n.lostFrames == 0);  // the repeat's seq was accounted: no false gap after it
        CHECK(n.staleFrames == 0);
        CHECK(n.linkLostFrames == 0);
        CHECK(n.endDisagreements == 0);
        CHECK(n.sessionsLinkLossUnknown == 0);
        CHECK(repeat.latestSession().linkLossKnown);
        CHECK(repeat.latestSession().lostFrames == 0);
    }

    SessionInfo other = standardInfo();
    other.routineId = "blueRightShort";
    c.add(s2::FrameType::Hello, helloPayload(other));  // different, seq 4
    c.add(s2::FrameType::Tick, tickPayload(record(2.0)));
    s2::Shul2Reader mismatch{c.bytes};
    CHECK(readAll(mismatch).size() == 6);
    CHECK(mismatch.counters().sessions == 2);
    CHECK(mismatch.counters().helloMismatches == 1);
    CHECK(mismatch.counters().helloRepeats == 1);
    CHECK(mismatch.counters().lateJoins == 0);  // a mismatch is not a late join
    CHECK(mismatch.counters().lostFrames == 0);
    CHECK(mismatch.latestSession().hello.routineId == "blueRightShort");
    CHECK(mismatch.latestSession().lateJoin);  // not read from its seq-0 Hello
    CHECK(mismatch.counters().sessionsLinkLossUnknown == 2);  // neither session saw its End
}

// Would catch: review mutations R5/N1 at a writer's real cadence. H1 Part A2's sink re-sends its
// Hello every 100 frames by default (a writer policy), so EVERY real capture goes through the
// repeat path, and linkLostFrames is the reader's headline figure for the link: a repeat that
// skipped the sequence rule would report ten lost frames, all "lost by the link", on this
// perfect capture of 1,000 ticks.
TEST_CASE("SHUL/2 reader: a clean stream with a re-Hello every 100 frames loses nothing") {
    const Bytes hello = helloPayload();
    Capture c;
    c.add(s2::FrameType::Hello, hello);
    std::uint32_t sinceHello = 0;
    std::uint32_t reHellos = 0;
    const auto reHelloIfDue = [&] {
        if (sinceHello == 100) {  // once 100 frames followed the last Hello, before the next frame
            c.add(s2::FrameType::Hello, hello);
            ++reHellos;
            sinceHello = 0;
        }
    };
    for (int i = 0; i < 1000; ++i) {
        reHelloIfDue();
        c.add(s2::FrameType::Tick, tickPayload(record(0.001 * i)));
        ++sinceHello;
    }
    reHelloIfDue();
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    REQUIRE(reHellos == 10);  // at seqs 101, 202, …, 1010

    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(frames.size() == c.starts.size());
    const auto& n = reader.counters();
    CHECK(n.sessions == 1);
    CHECK(n.helloRepeats == reHellos);
    CHECK(n.helloMismatches == 0);
    CHECK(n.lostFrames == 0);
    CHECK(n.staleFrames == 0);
    CHECK(n.linkLostFrames == 0);
    CHECK(n.endDisagreements == 0);
    CHECK(reader.latestSession().linkLossKnown);
    CHECK(reader.latestSession().linkLostFrames == 0);
}

// Would catch: a frame after an End delivered as if the session were still open.
TEST_CASE("SHUL/2 reader: a frame after an End and before the next Hello is outside any session") {
    Capture c = standardSession(3);
    c.add(s2::FrameType::Tick, tickPayload(record(9.0)));
    c.addAt(typeByte(s2::FrameType::Hello), 0, helloPayload());  // the writer re-opened
    c.add(s2::FrameType::Tick, tickPayload(record(10.0)));
    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    CHECK(frames.size() == c.starts.size() - 1);
    CHECK(reader.counters().framesOutsideSession == 1);
    CHECK(reader.counters().sessions == 2);
    CHECK(reader.counters().sessionsEnded == 1);
    CHECK_FALSE(reader.latestSession().lateJoin);
}

// ── Versions ────────────────────────────────────────────────────────────────────────

// Would catch: an unknown FUTURE minor refused (mutation 7), its appended bytes not counted
// (11 — and review mutation N8: not counted on a RE-Hello), a newer-minor Tick decoded from its
// whole payload (10), a Log's fixed width read as the bytes BEFORE the text (12). A hand-built
// 1.7 stream: its Hello is 272 bytes and declares every width +16 (logFixedBytes 30, the 16 after
// the text), and every frame carries 16 appended bytes — its periodic re-Hello included.
TEST_CASE("SHUL/2 reader: an unknown future 1.7 stream is read — its prefix bit-identical, 16 bytes ignored per frame") {
    const s2::WidthRow row17{272, 444, 184, 48, 30};
    RunSummary sum;
    sum.motionsStarted = 3;
    sum.blackboxDropped = 9;
    sum.setRoutineId("future");
    Capture c;
    c.add(s2::FrameType::Hello, helloAt(1, 7, row17));
    for (int i = 0; i < 5; ++i) {
        c.add(s2::FrameType::Tick, appended(tickPayload(record(1.0 * i)), 16));
    }
    c.add(s2::FrameType::Hello, helloAt(1, 7, row17));  // the writer's re-Hello, byte-identical
    c.add(s2::FrameType::Log, appended(logPayload(1, "SEQ", "from the future", 7.5), 16));
    c.add(s2::FrameType::Log, appended(logPayload(2, "", "", 8.5), 16));
    c.add(s2::FrameType::Summary, appended(summaryPayload(sum), 16));
    c.add(s2::FrameType::End, appended(endPayloadAt(c.nextSeq), 16));
    REQUIRE(c.starts.size() == 11);

    s2::Shul2Reader reader{c.bytes};
    const auto frames = readAll(reader);
    REQUIRE(frames.size() == 11);
    CHECK(reader.latestSession().verdict == s2::Verdict::NewerMinor);
    CHECK(reader.latestSession().hello.versionMinor == 7);
    CHECK(reader.counters().refusedSessions == 0);
    CHECK(reader.counters().wrongWidthSkipped == 0);
    CHECK(reader.counters().helloRepeats == 1);
    CHECK(reader.counters().extensionBytesIgnored == 16U * 11U);  // both Hellos included
    CHECK(reader.status() == s2::CaptureStatus::Ok);

    for (int i = 0; i < 5; ++i) {
        DebugRecord out;
        bool corrupt = false;
        REQUIRE(s2::decodeTick(frames[static_cast<std::size_t>(1 + i)].payload, out, corrupt));
        CHECK_FALSE(corrupt);
        checkSameRecordBits(record(1.0 * i), out);
    }
    CHECK(frames[6].type == s2::FrameType::Hello);
    s2::LogLine l;
    REQUIRE(s2::decodeLog(frames[7].payload, l));
    CHECK(l.level == 1);
    CHECK(l.tag == "SEQ");
    CHECK(l.message == "from the future");
    CHECK(l.t == 7.5);
    CHECK(l.nonTextBytes == 30);
    RunSummary out;
    std::uint32_t dropped = 0;
    REQUIRE(s2::decodeSummary(frames[9].payload, out, dropped));
    CHECK(out.motionsStarted == 3);
    CHECK(dropped == 9);
    CHECK(out.routineId() == "future");
    s2::EndInfo e;
    REQUIRE(s2::decodeEnd(frames[10].payload, e));
    CHECK(e.framesOffered == 10);
    CHECK(reader.latestSession().linkLossKnown);
    CHECK(reader.counters().lostFrames == 0);
}

// Would catch: another major accepted (mutation 6) or a refused session's frames delivered
// (mutation 18); a refused session's identical re-Hello taken as a NEW (refused) session (review
// mutation N6 — the session "stays open" until a Hello that differs); a refused Hello's bytes 12
// onward read with 1.0's layout into its report (review, correctness lens); and a capture whose
// LATEST session was refused called Refused although an earlier one was delivered (review
// mutation N32 — Ok means at least one session was accepted). The 2.0 Hello keeps only what
// every major keeps — the frozen bytes 0-11 — and everything after them differs; its frames are
// of types and widths 1.0 never defined.
TEST_CASE("SHUL/2 reader: a 2.0 session is refused whole; a 1.0 session after it is delivered") {
    Bytes hello20 = helloPayload();
    hello20.resize(40);
    putLe(hello20, 8, 2, 2);  // versionMajor 2
    for (std::size_t i = 12; i < hello20.size(); ++i) {
        hello20[i] = static_cast<std::byte>((0xA0U + i) & 0xFFU);
    }
    Capture c;
    c.add(s2::FrameType::Hello, hello20);
    c.add(s2::FrameType::Tick, Bytes(100, std::byte{0x31}));
    c.add(s2::FrameType::Log, Bytes(9, std::byte{0x32}));
    c.addAt(99, c.nextSeq, Bytes(20, std::byte{0x33}));
    c.add(s2::FrameType::End, Bytes(5, std::byte{0x34}));
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));  // after its "End": still its frames

    s2::Shul2Reader refused{c.bytes};
    CHECK(readAll(refused).empty());
    CHECK(refused.status() == s2::CaptureStatus::Refused);
    CHECK(refused.latestSession().verdict == s2::Verdict::RefusedMajor);
    CHECK(refused.latestSession().hello.versionMajor == 2);  // it still NAMES itself
    CHECK(refused.counters().framesInRefusedSessions == 5);
    CHECK(refused.counters().sessionsLinkLossUnknown == 0);  // a refused session has no link figure

    // The more dangerous 2.0: one that kept 1.0's Hello layout and frame widths, so nothing but
    // the major tells it apart. Refused whole all the same — the major is the only word on it.
    Bytes lookalike = helloPayload();
    putLe(lookalike, 8, 2, 2);
    Capture l;
    l.add(s2::FrameType::Hello, lookalike);
    l.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    l.add(s2::FrameType::Hello, lookalike);  // its writer's identical re-Hello, at seq 2
    l.add(s2::FrameType::Log, logPayload(2, "MOT", "not yours to read", 1.0));
    l.add(s2::FrameType::End, endPayloadAt(4));
    s2::Shul2Reader lookalikeReader{l.bytes};
    CHECK(readAll(lookalikeReader).empty());
    CHECK(lookalikeReader.status() == s2::CaptureStatus::Refused);
    CHECK(lookalikeReader.latestSession().verdict == s2::Verdict::RefusedMajor);
    CHECK(lookalikeReader.counters().sessions == 1);  // the re-Hello repeats the refused session
    CHECK(lookalikeReader.counters().refusedSessions == 1);
    CHECK(lookalikeReader.counters().helloRepeats == 1);
    CHECK(lookalikeReader.counters().lateJoins == 0);
    CHECK(lookalikeReader.counters().framesInRefusedSessions == 4);  // Tick, re-Hello, Log, End
    CHECK_FALSE(lookalikeReader.latestSession().sawEnd);
    // Its report NAMES it — and shows nothing of its bytes 12 onward, which are 2.0's to define.
    const s2::Hello& refusedHello = lookalikeReader.latestSession().hello;
    CHECK(refusedHello.versionMajor == 2);
    CHECK(refusedHello.payloadBytes == 256);
    CHECK_FALSE(refusedHello.complete);
    CHECK(refusedHello.routineId.empty());
    CHECK(refusedHello.buildHash.empty());
    CHECK(refusedHello.declared.tick == 0);
    CHECK(bitsOf(refusedHello.epochSeconds) == 0ULL);

    const Capture ok = standardSession(4);
    c.raw(ok.bytes);
    s2::Shul2Reader both{c.bytes};
    const auto frames = readAll(both);
    CHECK(frames.size() == ok.starts.size());
    CHECK(both.counters().sessions == 2);
    CHECK(both.counters().refusedSessions == 1);
    CHECK(both.status() == s2::CaptureStatus::Ok);
    CHECK(both.latestSession().verdict == s2::Verdict::Exact);

    // The other order: an accepted session, then its writer re-opened under a major this build
    // cannot read. The capture's frames WERE delivered, so it is Ok, whatever the latest says.
    Capture okThenRefused = standardSession(4);
    okThenRefused.addAt(typeByte(s2::FrameType::Hello), 0, lookalike);
    okThenRefused.add(s2::FrameType::Tick, tickPayload(record(9.0)));
    s2::Shul2Reader later{okThenRefused.bytes};
    CHECK(readAll(later).size() == ok.starts.size());
    CHECK(later.counters().sessions == 2);
    CHECK(later.counters().refusedSessions == 1);
    CHECK(later.counters().framesInRefusedSessions == 1);
    CHECK(later.latestSession().verdict == s2::Verdict::RefusedMajor);
    CHECK(later.status() == s2::CaptureStatus::Ok);
}

// Would catch: an older minor's widths checked against the wrong row (mutation 9) at the READER,
// and the known widths for extension counting taken from the wrong row. This build's capture
// read by a hypothetical 1.1 reader is OlderMinor with nothing ignored; a 1.1 stream read by this
// 1.0 build is NewerMinor with 8 bytes ignored per frame; the same 1.1 stream read by the 1.1
// reader is Exact with none.
TEST_CASE("SHUL/2 reader: older and newer minors across a hypothetical 1.1 reader") {
    const Capture v10 = standardSession(4);
    s2::Shul2Reader older{v10.bytes, kReader11};
    CHECK(readAll(older).size() == v10.starts.size());
    CHECK(older.latestSession().verdict == s2::Verdict::OlderMinor);
    CHECK(older.counters().extensionBytesIgnored == 0);

    Capture v11;
    v11.add(s2::FrameType::Hello, helloAt(1, 1, kRow11));
    v11.add(s2::FrameType::Tick, appended(tickPayload(record(0.0)), 8));
    v11.add(s2::FrameType::Log, appended(logPayload(2, "T", "m", 1.0), 8));
    v11.add(s2::FrameType::End, appended(endPayloadAt(3), 8));
    s2::Shul2Reader newer{v11.bytes};
    CHECK(readAll(newer).size() == 4);
    CHECK(newer.latestSession().verdict == s2::Verdict::NewerMinor);
    CHECK(newer.counters().extensionBytesIgnored == 4U * 8U);
    s2::Shul2Reader exact{v11.bytes, kReader11};
    CHECK(readAll(exact).size() == 4);
    CHECK(exact.latestSession().verdict == s2::Verdict::Exact);
    CHECK(exact.counters().extensionBytesIgnored == 0);

    // A 1.0 stream whose widths are NOT the 1.0 row is refused by the 1.1 reader.
    Capture wrong;
    wrong.add(s2::FrameType::Hello, helloAt(1, 0, kRow11));
    wrong.add(s2::FrameType::Tick, appended(tickPayload(record(0.0)), 8));
    s2::Shul2Reader r{wrong.bytes, kReader11};
    CHECK(readAll(r).empty());
    CHECK(r.latestSession().verdict == s2::Verdict::LayoutMismatch);
}

// ── Live reading and the chunking property ──────────────────────────────────────────

// Would catch: a live reader that skips the claimed length after a stall (mutation 3: the
// frames inside it would be lost), and a scanFrame that gives up before the whole candidate is
// held. A length corrupted to 4000 on a live stream: NeedMore until 14 + 4000 bytes are held,
// then a CRC failure, and every frame inside those bytes is recovered.
TEST_CASE("SHUL/2 live: a length corrupted to 4000 stalls until 4014 bytes, then its frames are recovered") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    c.add(s2::FrameType::Tick, tickPayload(record(0.0)));
    c.add(s2::FrameType::Tick, tickPayload(record(1.0)));  // frame 2: its length is corrupted
    for (int i = 2; i < 14; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    Bytes b = c.bytes;
    const std::size_t p = c.starts[2];
    putLe(b, p + 8, 4000, 2);
    REQUIRE(b.size() - p >= 4014);

    const std::span<const std::byte> all{b};
    for (std::size_t held = 10; held < 4014; ++held) {
        if (s2::scanFrame(all.subspan(p, held)).kind != s2::ScanKind::NeedMore) {
            FAIL_CHECK("not NeedMore at " << held << " bytes held");
            break;
        }
    }
    const s2::ScanResult atFull = s2::scanFrame(all.subspan(p, 4014));
    CHECK(atFull.kind == s2::ScanKind::Skip);
    CHECK(atFull.reason == s2::SkipReason::CrcMismatch);

    Reassembler live;
    for (const std::byte x : b) {
        live.feed(std::span<const std::byte>{&x, 1});
    }
    live.finish();
    std::vector<std::uint32_t> got;
    for (const auto& f : live.frames) {
        got.push_back(f.seq);
        // A frame that starts inside the stalled candidate is emitted only once the candidate
        // has been given all its 4014 bytes.
        const std::size_t idx = f.seq;  // seq i is frame i in this capture
        if (c.starts[idx] > p && c.starts[idx] < p + 4014) {
            CHECK(f.fedWhenEmitted >= p + 4014);
        }
    }
    std::vector<std::uint32_t> want = c.seqs;
    want.erase(want.begin() + 2);
    CHECK(got == want);  // every frame inside those 4000 bytes, recovered
    CHECK(live.crcFailures >= 1);
}

// Would catch: a live reader that never applies the tail rule when its stream ENDS (review
// finding, correctness lens: the contract said so only for a whole capture). The brief's named
// tail case, fed live one byte at a time: the corrupted length (428 → 2476, past the end of the
// stream) holds the two later Ticks AND the End for as long as the stream is open — nothing
// could release them but more bytes — and the tail rule at the end of the stream delivers them,
// exactly as the whole-capture reader does.
TEST_CASE("SHUL/2 live: a corrupted length near the end holds its frames until the stream ends; the tail rule then delivers them") {
    Capture c;
    c.add(s2::FrameType::Hello, helloPayload());
    for (int i = 0; i < 5; ++i) {
        c.add(s2::FrameType::Tick, tickPayload(record(static_cast<double>(i))));
    }
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    Bytes b = c.bytes;
    putLe(b, c.starts[3] + 8, 2476, 2);  // the third frame before the End
    REQUIRE(b.size() - c.starts[3] < 14 + 2476);

    Reassembler live;
    for (const std::byte x : b) {
        live.feed(std::span<const std::byte>{&x, 1});
    }
    std::vector<std::uint32_t> held;
    for (const auto& f : live.frames) {
        held.push_back(f.seq);
    }
    CHECK(held == std::vector<std::uint32_t>{0, 1, 2});  // the stream is open: it waits, End and all
    CHECK(live.buf.size() - live.head == b.size() - c.starts[3]);  // everything from the bad length on

    live.finish();  // the stream ended: what is held is a whole capture — the tail rule
    std::vector<std::uint32_t> got;
    for (const auto& f : live.frames) {
        got.push_back(f.seq);
    }
    CHECK(got == std::vector<std::uint32_t>{0, 1, 2, 4, 5, 6});
    s2::Shul2Reader whole{b};
    CHECK(got == seqsOf(readAll(whole)));
}

// Would catch: a whole-capture reader and a live reader that disagree about ANY frame in ANY
// chunking — which would make a recording and its live stream tell different stories. A
// scripted stream with a garbage prefix, a CRC-damaged frame, a length corrupted to 4000
// mid-stream and a cut tail, fed through scanFrame in every chunk size 1-64 and in seeded random
// splits, with the whole-capture tail rule applied at the end of input.
TEST_CASE("SHUL/2 live: the chunking property — every chunking gives the whole-capture reader's frames") {
    Capture c;
    c.raw({std::byte{0x11}, std::byte{0xFE}, std::byte{0x53}, std::byte{0x02}, std::byte{0x00},
           std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x20},
           std::byte{0x00}, std::byte{0xFE}, std::byte{0x22}});
    c.add(s2::FrameType::Hello, helloPayload());
    std::size_t crcDamaged = 0;     // the frame index of tick 4: one payload bit flipped
    std::size_t lengthDamaged = 0;  // the frame index of tick 8: its length set to 4000
    for (int i = 0; i < 20; ++i) {
        if (i == 4) {
            crcDamaged = c.starts.size();
        }
        if (i == 8) {
            lengthDamaged = c.starts.size();
        }
        c.add(s2::FrameType::Tick, tickPayload(record(1.0 * i)));
        if (i % 7 == 3) {
            c.add(s2::FrameType::Log, logPayload(2, "MOT", "chunk", 1.0 * i));
        }
    }
    c.add(s2::FrameType::Summary, summaryPayload(RunSummary{}));
    c.add(s2::FrameType::End, endPayloadAt(c.nextSeq));
    Bytes b = c.bytes;
    b[c.starts[crcDamaged] + 100] ^= std::byte{0x40};
    putLe(b, c.starts[lengthDamaged] + 8, 4000, 2);
    REQUIRE(b.size() - c.starts[lengthDamaged] >= 4014);
    const Bytes cutTail = frameOf(2, 999, tickPayload(record(0.0)));
    b.insert(b.end(), cutTail.begin(), cutTail.begin() + 100);

    s2::Shul2Reader whole{b};
    const auto want = readAll(whole);
    REQUIRE(want.size() == c.starts.size() - 2);
    REQUIRE(whole.counters().truncated);

    const auto sameAsWhole = [&](const Reassembler& live) {
        if (live.frames.size() != want.size() || live.skipped != whole.counters().bytesSkipped
            || live.crcFailures != whole.counters().crcFailures) {
            return false;
        }
        for (std::size_t i = 0; i < want.size(); ++i) {
            if (live.frames[i].seq != want[i].seq || live.frames[i].type != typeByte(want[i].type)
                || live.frames[i].payload != want[i].payload) {
                return false;
            }
        }
        return true;
    };
    const std::span<const std::byte> all{b};
    for (std::size_t chunk = 1; chunk <= 64; ++chunk) {
        Reassembler live;
        for (std::size_t at = 0; at < all.size(); at += chunk) {
            live.feed(all.subspan(at, std::min(chunk, all.size() - at)));
        }
        live.finish();
        CAPTURE(chunk);
        CHECK(sameAsWhole(live));
    }
    std::mt19937 rng{0x5EED5EEDU};
    for (int trial = 0; trial < 16; ++trial) {
        Reassembler live;
        std::size_t at = 0;
        while (at < all.size()) {
            const std::size_t n = std::min<std::size_t>(std::uniform_int_distribution<std::size_t>{1, 700}(rng),
                                                        all.size() - at);
            live.feed(all.subspan(at, n));
            at += n;
        }
        live.finish();
        CAPTURE(trial);
        CHECK(sameAsWhole(live));
    }
}
