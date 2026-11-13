// The CALIBRATION RECORD — the on-card format (chunk R3d Part 2).
//
// NO HARDWARE CLAIM: no record in this file has ever been written to a physical SD card.
// Everything here is bytes in memory and, in the ProsBlockSource/Sink test, bytes in a real
// temp directory on the host.
//
// ── Why the golden array is hand-derived, and how ───────────────────────────────────
// A round trip proves the encoder and decoder AGREE. It cannot prove either is RIGHT — two
// halves that move together agree perfectly, which is exactly what E1's mutation 1b showed
// for the blackbox. So the bytes below were derived from the LAYOUT TABLE in
// calibration_record.hpp by a second, independent implementation (Python `struct.pack` +
// `zlib.crc32`, the textbook CRC-32/IEEE the header promises to compute), and every group of
// bytes is commented with the field it spells so a human can check it against the table by
// eye. Nothing here was captured from encodeRecord(). Move a field in BOTH the encoder and
// the decoder and the round-trip tests stay green while THIS one goes red — which is the
// mutation the brief requires and the reason the array exists.
//
// The chosen values are deliberately checkable by hand: 1.375 = 0x3FF6000000000000,
// 1.5 = 0x3FF8000000000000, 13.25 = 0x402A800000000000, and robot two's measured signed
// ports as two's-complement bytes (-11 = 0xF5, 12 = 0x0C, …).

#include "doctest.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "shulib/calibration/calibration_record.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_block_source.hpp"
#include "shulib/units/quantity.hpp"

using shulib::calibration::crc32Ieee;
using shulib::calibration::decodeRecord;
using shulib::calibration::encodeRecord;
using shulib::calibration::kRecordBytes;
using shulib::calibration::kRecordPayloadBytes;
using shulib::calibration::kRecordPrefixBytes;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::units::Length;

namespace {

/// The description the golden bytes spell: robot two's measured ports and signs, a LEFT
/// scale of 1.375 in/rad, a RIGHT scale of 1.5 (deliberately DIFFERENT, so a test that
/// swapped the two sides could not pass), a 13.25 in track width, RobotMeasured.
DrivetrainDescription goldenDescription() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[5] = {-11, 12, -13, 14, -15};
    const std::int8_t right[5] = {20, -19, 18, -17, 16};
    for (int i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = Length{1.375};
    d.right.inchesPerRadian = Length{1.5};
    d.trackWidth = Length{13.25};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "R3d Part 2 golden");
    return d;
}

/// THE GOLDEN BYTES — derived from the layout table, not from the encoder (file header).
///
/// The bound is DEDUCED, never written: a `std::array<std::uint8_t, 180>` silently ZERO-FILLS
/// a short initializer list, so a miscounted pad run would have produced a 180-byte array
/// whose tail was zeros and whose real content was shifted — which is precisely what happened
/// on the first draft of this file (the CRC landed at offset 160). With the bound deduced, the
/// static_assert below turns that class of typo into a COMPILE error instead of a confusing
/// byte-compare failure sixteen bytes downstream.
constexpr std::uint8_t kGolden[] = {
    // offset 0: magic "SHDC"
    0x53, 0x48, 0x44, 0x43,
    // offset 4: version u16 LE = 1
    0x01, 0x00,
    // offset 6: payloadBytes u16 LE = 168 = 0x00A8
    0xA8, 0x00,
    // offset 8: kind u8 = Tank(1); cartridge u8 = Blue(3); imuPort u8 = 2
    0x01, 0x03, 0x02,
    // offset 11: left.signedPorts[21] — -11 12 -13 14 -15, then 16 unused zeros
    0xF5, 0x0C, 0xF3, 0x0E, 0xF1,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // offset 32: left.count u8 = 5
    0x05,
    // offset 33: left.inchesPerRadian f64 LE = 1.375 = 0x3FF6000000000000
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6, 0x3F,
    // offset 41: right.signedPorts[21] — 20 -19 18 -17 16, then 16 unused zeros
    0x14, 0xED, 0x12, 0xEF, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // offset 62: right.count u8 = 5
    0x05,
    // offset 63: right.inchesPerRadian f64 LE = 1.5 = 0x3FF8000000000000
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F,
    // offset 71: trackWidth f64 LE = 13.25 = 0x402A800000000000
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x2A, 0x40,
    // offset 79: source u8 = RobotMeasured(2)
    0x02,
    // offset 80: provenance[96] — "R3d Part 2 golden" (17 chars), NUL-padded
    0x52, 0x33, 0x64, 0x20, 0x50, 0x61, 0x72, 0x74, 0x20, 0x32, 0x20,  // "R3d Part "
    0x67, 0x6F, 0x6C, 0x64, 0x65, 0x6E,                                // "golden" (17 chars)
    // 79 NUL pad bytes (17 + 79 = 96)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // offset 176: CRC-32/IEEE u32 LE over bytes 0..175 = 0x45C46E98
    //   (Python: zlib.crc32(bytes(record[:176])) — an implementation this header has
    //    never seen, which is the whole point of a golden)
    0x98, 0x6E, 0xC4, 0x45,
};

static_assert(sizeof kGolden == 180,
              "the golden array must be exactly one 180-byte record — count the pad runs");

std::vector<std::byte> goldenBytes() {
    std::vector<std::byte> v;
    v.reserve(sizeof kGolden);
    for (const std::uint8_t b : kGolden) {
        v.push_back(static_cast<std::byte>(b));
    }
    return v;
}

}  // namespace

TEST_CASE("CalibrationRecord 1: the GOLDEN bytes — the layout is a document, not an agreement") {
    // BUG CAUGHT (brief §4 test 1's mutation): a field moved in BOTH the encoder and the
    // decoder. Every round trip below stays green; this case goes red, because these bytes
    // came from the layout TABLE via an independent implementation, not from encodeRecord().
    static_assert(sizeof kGolden == kRecordBytes, "the golden array must be a whole record");
    CHECK(kRecordBytes == 180);
    CHECK(kRecordPayloadBytes == 168);
    CHECK(kRecordPrefixBytes == 8);

    const DrivetrainDescription d = goldenDescription();
    char why[192];
    REQUIRE_MESSAGE(d.valid(why, sizeof why), why);

    std::array<std::byte, kRecordBytes> out{};
    REQUIRE(encodeRecord(d, out) == kRecordBytes);
    for (std::size_t i = 0; i < kRecordBytes; ++i) {
        CHECK_MESSAGE(static_cast<std::uint8_t>(out[i]) == kGolden[i],
                      "byte ", i, " of the record");
    }

    // And the CRC this header computes is the one an independent CRC-32/IEEE produced.
    CHECK(crc32Ieee(std::span<const std::byte>{out}.subspan(0, 176)) == 0x45C46E98U);

    // The golden bytes decode back to the same description, field by field — so the array is
    // a two-way pin, not just an encoder snapshot.
    const std::vector<std::byte> bytes = goldenBytes();
    DrivetrainDescription back{};
    REQUIRE_MESSAGE(decodeRecord(bytes, back, why, sizeof why), why);
    CHECK(back.kind == DrivetrainKind::Tank);
    CHECK(back.cartridge == MotorCartridge::Blue);
    CHECK(back.imuPort == 2);
    CHECK(static_cast<int>(back.left.count) == 5);
    CHECK(static_cast<int>(back.right.count) == 5);
    CHECK(back.left.signedPorts[0] == -11);
    CHECK(back.left.signedPorts[4] == -15);
    CHECK(back.right.signedPorts[0] == 20);
    CHECK(back.right.signedPorts[3] == -17);
    CHECK(back.left.inchesPerRadian.value() == 1.375);
    CHECK(back.right.inchesPerRadian.value() == 1.5);   // the sides are NOT interchangeable
    CHECK(back.trackWidth.value() == 13.25);
    CHECK(back.source == DescriptionSource::RobotMeasured);
    CHECK(std::string{back.provenance} == "R3d Part 2 golden");
}

TEST_CASE("CalibrationRecord 1b: CRC-32/IEEE against published test vectors") {
    // The golden's CRC is only as good as the CRC function. These are the standard
    // CRC-32/IEEE check values, which every conforming implementation produces — so a
    // mutation of the polynomial, the init value or the final inversion fails HERE, naming
    // the arithmetic rather than a byte offset 176 bytes into a record.
    const auto crcOf = [](const char* s) {
        return crc32Ieee(std::as_bytes(std::span<const char>{s, std::strlen(s)}));
    };
    CHECK(crcOf("") == 0x00000000U);
    CHECK(crcOf("a") == 0xE8B7BE43U);
    CHECK(crcOf("abc") == 0x352441C2U);
    CHECK(crcOf("123456789") == 0xCBF43926U);  // the canonical "check" value
    CHECK(crcOf("The quick brown fox jumps over the lazy dog") == 0x414FA339U);
}

TEST_CASE("CalibrationRecord 2: every decode refusal, each the SOLE cause, each named") {
    // BUG CAUGHT (brief §4 test 2's mutations: drop the CRC check; drop the length check).
    // A decoder that guesses is worse than one that refuses: the guess drives a robot.
    const DrivetrainDescription good = goldenDescription();
    std::array<std::byte, kRecordBytes> ref{};
    REQUIRE(encodeRecord(good, ref) == kRecordBytes);

    char why[192];
    DrivetrainDescription out{};

    SUBCASE("bad magic") {
        auto bytes = ref;
        bytes[2] = std::byte{'X'};
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("magic") != std::string::npos);
    }

    SUBCASE("version 2 — a layout this build was not written for") {
        auto bytes = ref;
        bytes[4] = std::byte{2};
        // The CRC now mismatches too; the VERSION check must fire FIRST, because
        // "recalibrate with the new build" and "your card is corrupt" are different errands.
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("version 2") != std::string::npos);
    }

    SUBCASE("a payload length from another layout") {
        auto bytes = ref;
        bytes[6] = std::byte{0xA9};  // 169, one more than v1's 168
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("payload") != std::string::npos);
    }

    SUBCASE("SHORT buffer — truncated") {
        std::vector<std::byte> bytes(ref.begin(), ref.end() - 1);
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("truncated") != std::string::npos);
        CHECK(std::string{why}.find("179") != std::string::npos);
    }

    SUBCASE("LONG buffer — padded (a file appended to, or a second record concatenated)") {
        std::vector<std::byte> bytes(ref.begin(), ref.end());
        bytes.push_back(std::byte{0});
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("181") != std::string::npos);
    }

    SUBCASE("a buffer too short even for the header") {
        std::vector<std::byte> bytes(4, std::byte{0});
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("too short") != std::string::npos);
    }

    SUBCASE("ONE BIT flipped in each field class — every one caught by the CRC") {
        // The offsets below are the layout table's, one per KIND of field: a sign byte, a
        // count, a scale's mantissa, a scale's EXPONENT, the track width, the source byte and
        // the provenance text. Each is the sole change; each must refuse.
        struct Spot { std::size_t at; const char* what; };
        const Spot spots[] = {
            {11, "a LEFT sign byte"},        {32, "the LEFT count"},
            {33, "the LEFT scale's mantissa"}, {40, "the LEFT scale's EXPONENT"},
            {41, "a RIGHT sign byte"},        {63 + 7, "the RIGHT scale's exponent"},
            {71 + 7, "the track width's exponent"}, {79, "the source byte"},
            {80, "the provenance's first character"}, {176, "the CRC itself"},
        };
        for (const Spot& s : spots) {
            auto bytes = ref;
            bytes[s.at] = static_cast<std::byte>(static_cast<std::uint8_t>(bytes[s.at]) ^ 0x01U);
            DrivetrainDescription decoded{};
            // std::string{}, not the bare pointer: doctest stringifies a `const char*` member
            // as an ADDRESS, and a failure message reading "one flipped bit in 0x562cd873..."
            // is a message nobody can act on (observed while running this file's mutations).
            const std::string what{s.what};
            CHECK_MESSAGE(decodeRecord(bytes, decoded, why, sizeof why) == false,
                          "one flipped bit in ", what, " was NOT refused");
            CHECK_MESSAGE(std::string{why}.find("CRC-32 mismatch") != std::string::npos,
                          "the refusal for ", what, " did not name the CRC: ", why);
        }
    }

    SUBCASE("intact bytes spelling an INVALID description — refused, and said to be intact") {
        // A producer bug, not a card problem. The message must separate the two, because the
        // operator's next move differs: recalibrate vs. report a bug.
        DrivetrainDescription bad = good;
        bad.left.count = 0;  // a side nobody found
        std::array<std::byte, kRecordBytes> bytes{};
        REQUIRE(encodeRecord(bad, bytes) == kRecordBytes);
        CHECK_FALSE(decodeRecord(bytes, out, why, sizeof why));
        CHECK(std::string{why}.find("CRC ok") != std::string::npos);
        CHECK(std::string{why}.find("LEFT side has no ports") != std::string::npos);
    }

    SUBCASE("a REFUSED decode leaves the caller's description untouched") {
        // The boot path decodes into the description it is about to build a graph from. A
        // decoder that half-fills it on the way to a refusal is a graph built from rubble.
        DrivetrainDescription target{};
        target.imuPort = 7;
        target.left.count = 3;
        auto bytes = ref;
        bytes[100] = static_cast<std::byte>(static_cast<std::uint8_t>(bytes[100]) ^ 0xFFU);
        CHECK_FALSE(decodeRecord(bytes, target, why, sizeof why));
        CHECK(target.imuPort == 7);
        CHECK(static_cast<int>(target.left.count) == 3);
    }

    SUBCASE("a decode with no `why` buffer still refuses (nullptr is legal)") {
        auto bytes = ref;
        bytes[0] = std::byte{'Q'};
        CHECK_FALSE(decodeRecord(bytes, out));
        CHECK_FALSE(decodeRecord(bytes, out, nullptr, 0));
    }
}

TEST_CASE("CalibrationRecord 2b: the encoder is whole-or-nothing on a short buffer") {
    // A half-written record that happens to decode is the failure the length check exists
    // for; refusing to write it at all is cheaper than catching it later.
    const DrivetrainDescription d = goldenDescription();
    std::array<std::byte, kRecordBytes - 1> tooSmall{};
    CHECK(encodeRecord(d, tooSmall) == 0U);
    for (const std::byte b : tooSmall) {
        CHECK(b == std::byte{0});  // nothing was written
    }
    // An OVERSIZED buffer is fine: the record is written into its first kRecordBytes and the
    // rest is the caller's (the calibrate program reads back into a bigger buffer on purpose,
    // so that a LONGER file on the card shows up as a length refusal instead of silently
    // fitting).
    std::array<std::byte, kRecordBytes + 16> big{};
    CHECK(encodeRecord(d, big) == kRecordBytes);
}

TEST_CASE("CalibrationRecord 3: the record travels the block seams, and a SHORT read refuses") {
    // The seams (hal::IBlockSink / hal::IBlockSource) carry the bytes; the FORMAT carries the
    // meaning. This is the host-side rehearsal of exactly what the calibrate program does on
    // the brain: encode, write, flush, read back, decode, compare.
    const DrivetrainDescription d = goldenDescription();
    std::array<std::byte, kRecordBytes> encoded{};
    REQUIRE(encodeRecord(d, encoded) == kRecordBytes);

    shulib::hal::fake::FakeBlockSink sink;
    REQUIRE(sink.write(encoded));
    REQUIRE(sink.flush());
    REQUIRE(sink.size() == kRecordBytes);

    shulib::hal::fake::FakeBlockSource source{
        std::vector<std::byte>{sink.bytes().begin(), sink.bytes().end()}};
    char why[192];

    SUBCASE("a whole record round-trips byte for byte and decodes") {
        std::array<std::byte, kRecordBytes> readBack{};
        REQUIRE(source.readAll(readBack) == kRecordBytes);
        CHECK(std::memcmp(readBack.data(), encoded.data(), kRecordBytes) == 0);
        DrivetrainDescription back{};
        REQUIRE_MESSAGE(decodeRecord(readBack, back, why, sizeof why), why);
        CHECK(back.left.inchesPerRadian.value() == d.left.inchesPerRadian.value());
        CHECK(back.trackWidth.value() == d.trackWidth.value());
        CHECK(source.readCalls() == 1);
    }

    SUBCASE("a TRUNCATED source refuses BY NAME — the count is what makes that possible") {
        source.setShortRead(kRecordBytes - 3);
        std::array<std::byte, kRecordBytes> readBack{};
        const std::size_t n = source.readAll(readBack);
        CHECK(n == kRecordBytes - 3);
        DrivetrainDescription back{};
        CHECK_FALSE(decodeRecord(std::span<const std::byte>{readBack}.subspan(0, n), back, why,
                                 sizeof why));
        CHECK(std::string{why}.find("truncated") != std::string::npos);
    }

    SUBCASE("a REFUSING source reads nothing, and that is 'no record', not 'an empty record'") {
        source.setOpen(false);
        std::array<std::byte, kRecordBytes> readBack{};
        CHECK(source.readAll(readBack) == 0U);
        CHECK_FALSE(source.isOpen());
        DrivetrainDescription back{};
        CHECK_FALSE(decodeRecord(std::span<const std::byte>{readBack}.subspan(0, 0), back, why,
                                 sizeof why));
        CHECK(std::string{why}.find("too short") != std::string::npos);
    }

    SUBCASE("an OPEN but EMPTY source is distinguishable from a refusing one") {
        shulib::hal::fake::FakeBlockSource empty{};
        std::array<std::byte, kRecordBytes> readBack{};
        CHECK(empty.readAll(readBack) == 0U);
        CHECK(empty.isOpen());  // THE distinction: a card with no calibration on it
    }

    SUBCASE("a card that filled mid-write leaves a PREFIX, and the prefix is refused") {
        shulib::hal::fake::FakeBlockSink dying;
        dying.setCapacity(100);
        CHECK_FALSE(dying.write(encoded));          // the device said no
        CHECK(dying.size() == 100);                 // and kept what it took
        shulib::hal::fake::FakeBlockSource fromCard{
            std::vector<std::byte>{dying.bytes().begin(), dying.bytes().end()}};
        std::array<std::byte, kRecordBytes> readBack{};
        const std::size_t n = fromCard.readAll(readBack);
        DrivetrainDescription back{};
        CHECK_FALSE(decodeRecord(std::span<const std::byte>{readBack}.subspan(0, n), back, why,
                                 sizeof why));
        CHECK(std::string{why}.find("truncated") != std::string::npos);
    }
}
