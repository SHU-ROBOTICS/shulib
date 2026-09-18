// Adapter tests for ProsBlockSource THROUGH THE HOST SHIM + a real filesystem
// (chunk R3d Part 2). The usd_is_installed belief runs through the shim (HA-122);
// the FILE* path runs against REAL files — the same join+open+read code the robot
// runs against /usd/, pointed at a temp directory through the injectable mount root.
//
// What only these can prove: the /usd/ prefix logic on the READ side, the no-card and
// no-file refusals (which are DIFFERENT situations from an empty file and must stay
// distinguishable), byte-verbatim reads of a file the SINK wrote, and that a truncated
// file comes back as a SHORT COUNT rather than an error — the one thing that lets the
// record format refuse it by name.
//
// NO HARDWARE CLAIM: nothing here has touched a V5 brain or a physical SD card.

#include "doctest.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "pros/shim_control.hpp"

#include "shulib/calibration/calibration_record.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/block_source.hpp"

using shulib::PreconditionError;
using shulib::hal::pros::ProsBlockSink;
using shulib::hal::pros::ProsBlockSource;

namespace {

namespace fs = std::filesystem;

/// A fresh scratch directory (with the trailing '/' the adapters demand), unique per test
/// run so parallel/stale runs cannot collide. Its own name, not the sink test's, so the two
/// files cannot interfere.
std::string scratchRoot() {
    const fs::path dir = fs::temp_directory_path() / "shulib_r3d_blocksource";
    fs::create_directories(dir);
    return dir.string() + "/";
}

}  // namespace

TEST_CASE("ProsBlockSource: a file the SINK wrote comes back BYTE-IDENTICAL (the real fopen path)") {
    // BUG CAUGHT: a read path that translates bytes (text mode, "r" instead of "rb"), or a
    // path join that disagrees with the sink's — on the robot that is "shulib Teleop" reading
    // a file "shulib Calibrate" never wrote, which looks exactly like "no calibration yet".
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;  // opt IN — the default is no card
    const std::string root = scratchRoot();
    const fs::path expected = fs::path(root) / "rt.bin";
    fs::remove(expected);

    // Every byte value that a text-mode read or a naive line-based copy would mangle:
    // NUL, LF, CR, CR+LF, DEL, 0x1A (DOS end-of-file), 0xFF.
    const std::array<std::byte, 10> payload{
        std::byte{0x00}, std::byte{0x0A}, std::byte{0x0D}, std::byte{0x0D}, std::byte{0x0A},
        std::byte{0x1A}, std::byte{0x7F}, std::byte{0xFF}, std::byte{0x53}, std::byte{0x00}};
    {
        ProsBlockSink sink{"rt.bin", root.c_str()};
        REQUIRE(sink.isOpen());
        REQUIRE(sink.write(payload));
        REQUIRE(sink.flush());
    }  // destruction closes the FILE*

    ProsBlockSource source{"rt.bin", root.c_str()};
    REQUIRE(source.isOpen());
    CHECK(std::string{source.path()} == expected.string());
    std::array<std::byte, 64> into{};
    const std::size_t n = source.readAll(into);
    REQUIRE(n == payload.size());
    for (std::size_t i = 0; i < payload.size(); ++i) {
        CHECK(into[i] == payload[i]);
    }

    // The seam's "reads from the start" contract: a SECOND call gives the same bytes, not
    // zero (the adapter seeks to 0). A boot path that retried would otherwise see an empty
    // file and report "no calibration" on a card that has one.
    std::array<std::byte, 64> again{};
    CHECK(source.readAll(again) == payload.size());
    CHECK(std::memcmp(again.data(), into.data(), payload.size()) == 0);
}

TEST_CASE("ProsBlockSource: a WHOLE calibration record survives the sink→card→source trip") {
    // This is the calibrate program's Done path, rehearsed on the host against real files:
    // encode → write → flush → close → re-open → read back → decode → compare (ruling P5).
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    fs::remove(fs::path(root) / shulib::calibration::kRecordFileName);

    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[5] = {-11, 12, -13, 14, -15};
    const std::int8_t right[5] = {20, -19, 18, -17, 16};
    for (int i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = shulib::units::Length{1.3751};
    d.right.inchesPerRadian = shulib::units::Length{1.3749};
    d.trackWidth = shulib::units::Length{13.7};
    d.source = shulib::hal::DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "host rehearsal of the Done path");

    std::array<std::byte, shulib::calibration::kRecordBytes> encoded{};
    REQUIRE(shulib::calibration::encodeRecord(d, encoded)
            == shulib::calibration::kRecordBytes);
    {
        ProsBlockSink sink{shulib::calibration::kRecordFileName, root.c_str()};
        REQUIRE(sink.isOpen());
        REQUIRE(sink.write(encoded));
        REQUIRE(sink.flush());
    }

    ProsBlockSource source{shulib::calibration::kRecordFileName, root.c_str()};
    REQUIRE(source.isOpen());
    // Read into a buffer LARGER than the record on purpose: a longer file must show up as a
    // length refusal, never as "the first 180 bytes happened to fit".
    std::array<std::byte, shulib::calibration::kRecordBytes + 32> into{};
    const std::size_t n = source.readAll(into);
    REQUIRE(n == shulib::calibration::kRecordBytes);
    CHECK(std::memcmp(into.data(), encoded.data(), n) == 0);

    shulib::hal::DrivetrainDescription back{};
    char why[192];
    REQUIRE_MESSAGE(shulib::calibration::decodeRecord(
                        std::span<const std::byte>{into}.subspan(0, n), back, why, sizeof why),
                    why);
    CHECK(back.left.inchesPerRadian.value() == d.left.inchesPerRadian.value());
    CHECK(back.right.inchesPerRadian.value() == d.right.inchesPerRadian.value());
    CHECK(back.trackWidth.value() == d.trackWidth.value());
    CHECK(back.imuPort == 2);
    CHECK(std::string{back.provenance} == "host rehearsal of the Done path");
}

TEST_CASE("ProsBlockSource: NO CARD — construction succeeds, the source refuses, PROS is not asked") {
    // BUG CAUGHT (the read-side twin of the sink's mutation 11): the usd_is_installed()
    // probe removed. On the host the fopen would find the real file and the source would
    // report a healthy read with no card in the robot's slot — "shulib Teleop" would boot
    // from a record that is not on the robot. The shim's ADVERSARIAL default is no card;
    // this case never opts in.
    pros::shim::resetAll();
    const std::string root = scratchRoot();
    {
        pros::shim::usdState().installed = true;   // write a real file first...
        ProsBlockSink sink{"nocard.bin", root.c_str()};
        REQUIRE(sink.isOpen());
        const std::array<std::byte, 4> payload{std::byte{1}, std::byte{2}, std::byte{3},
                                               std::byte{4}};
        REQUIRE(sink.write(payload));
    }
    pros::shim::resetAll();                        // ...then take the card away

    ProsBlockSource source{"nocard.bin", root.c_str()};
    CHECK_FALSE(source.isOpen());
    std::array<std::byte, 8> into{};
    CHECK(source.readAll(into) == 0U);
    for (const std::byte b : into) {
        CHECK(b == std::byte{0});  // nothing was copied into the caller's buffer
    }
}

TEST_CASE("ProsBlockSource: a card with NO SUCH FILE refuses — and is NOT the same as an empty file") {
    // The distinction the composition root prints: "no SD card", "no calibration on this
    // card" and "a zero-length calibration file" send an operator to three different places.
    // isOpen() is what separates the first two from the third.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    fs::remove(fs::path(root) / "absent.bin");

    ProsBlockSource missing{"absent.bin", root.c_str()};
    CHECK_FALSE(missing.isOpen());
    std::array<std::byte, 8> into{};
    CHECK(missing.readAll(into) == 0U);

    // A file that EXISTS and is empty: open, and reads zero. Same count, different isOpen().
    {
        ProsBlockSink sink{"empty.bin", root.c_str()};
        REQUIRE(sink.isOpen());
    }
    ProsBlockSource empty{"empty.bin", root.c_str()};
    CHECK(empty.isOpen());
    CHECK(empty.readAll(into) == 0U);
}

TEST_CASE("ProsBlockSource: a TRUNCATED file comes back as a SHORT COUNT, not an error") {
    // The count is the whole reason readAll returns a size_t: a card yanked mid-write leaves
    // a prefix, and the record format must be able to say "truncated" rather than "corrupt".
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    {
        ProsBlockSink sink{"short.bin", root.c_str()};
        REQUIRE(sink.isOpen());
        const std::vector<std::byte> prefix(37, std::byte{0xC5});
        REQUIRE(sink.write(prefix));
        REQUIRE(sink.flush());
    }
    ProsBlockSource source{"short.bin", root.c_str()};
    REQUIRE(source.isOpen());
    std::array<std::byte, 180> into{};
    CHECK(source.readAll(into) == 37U);
    CHECK(into[36] == std::byte{0xC5});
    CHECK(into[37] == std::byte{0x00});  // nothing beyond the count was touched
}

TEST_CASE("ProsBlockSource: an EMPTY destination span reads nothing and does not touch the file") {
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    {
        ProsBlockSink sink{"anything.bin", root.c_str()};
        const std::array<std::byte, 2> payload{std::byte{1}, std::byte{2}};
        REQUIRE(sink.write(payload));
    }
    ProsBlockSource source{"anything.bin", root.c_str()};
    REQUIRE(source.isOpen());
    CHECK(source.readAll(std::span<std::byte>{}) == 0U);
}

TEST_CASE("ProsBlockSource: the /usd/ double-prefix mistake is a LOUD precondition, not a wrong path") {
    // BUG CAUGHT: a caller pre-prefixing "/usd/shulib_drivetrain.bin" (the vendored API
    // itself teaches two path conventions — HA-122): silently joined it becomes
    // "/usd//usd/…" and the source reports "no record" on a card that has one, forever.
    // The adapter owns the prefix; a leading '/' must refuse at the call site, loudly — and
    // it must refuse on the READ side too, or the two halves can disagree about the path.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    CHECK_THROWS_AS(ProsBlockSource("/usd/shulib_drivetrain.bin"), PreconditionError);
    CHECK_THROWS_AS(ProsBlockSource(""), PreconditionError);
    CHECK_THROWS_AS(ProsBlockSource(nullptr), PreconditionError);
    CHECK_THROWS_AS(ProsBlockSource("rec.bin", "/tmp"), PreconditionError);  // no trailing '/'
    CHECK_THROWS_AS(ProsBlockSource("rec.bin", ""), PreconditionError);
}

TEST_CASE("ProsBlockSource: the source and the sink agree about the path, by construction") {
    // ONE file name (calibration::kRecordFileName), ONE prefix rule, two adapters. If these
    // two strings ever differ, "shulib Calibrate" writes where "shulib Teleop" does not look
    // — and both programs report success.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsBlockSink sink{shulib::calibration::kRecordFileName, root.c_str()};
    ProsBlockSource source{shulib::calibration::kRecordFileName, root.c_str()};
    CHECK(std::string{sink.path()} == std::string{source.path()});
    CHECK(std::string{sink.path()}.ends_with("shulib_drivetrain.bin"));
}
