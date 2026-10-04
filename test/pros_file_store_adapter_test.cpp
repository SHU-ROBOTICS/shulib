// Adapter tests for ProsFileStore THROUGH THE HOST SHIM + a real filesystem (chunk SAFE-SAVE Part
// A). The card probe runs through the shim (HA-122); the files are REAL — the same ProsBlockSink /
// ProsBlockSource code the robot runs against /usd/, pointed at a temp directory through the
// injectable root.
//
// What only these can prove: that replace() TRUNCATES (a longer file becomes exactly the new
// bytes — the "wb" belief, HA-166 (1), as far as Linux can show it), that no card creates nothing,
// that a refused write and a refused flush each report their own step (against /dev/full), that a
// name that is not bare throws from BOTH methods, that awkward bytes survive, that the path
// in each result is the ADAPTER's (so "/usd/" stays spelled only in the two adapters), and that
// the whole safe save runs end to end against real files.
//
// WHAT THEY CANNOT PROVE (HA-166): Linux has rename(), remove() and a read-write open, so a save
// built on any of them would pass here and fail on the brain — which is why the platform scan
// (record_store_platform_scan_test.cpp) exists. NO HARDWARE CLAIM.

#include "doctest.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "pros/shim_control.hpp"

#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/file_store.hpp"
#include "shulib/hal/pros/file_store.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::ReplaceStep;
using shulib::hal::StoreRead;
using shulib::hal::StoreReplace;
using shulib::hal::pros::ProsFileStore;

namespace {

namespace fs = std::filesystem;

/// A fresh scratch directory of its own (with the trailing '/' the adapters demand), emptied, so
/// "no file is created" is checkable by listing it.
std::string scratchRoot() {
    const fs::path dir = fs::temp_directory_path() / "shulib_safesave_filestore";
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir.string() + "/";
}

std::vector<std::byte> fileBytes(const fs::path& p) {
    std::ifstream in{p, std::ios::binary};
    const std::vector<char> raw{std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
    std::vector<std::byte> out;
    for (const char c : raw) out.push_back(static_cast<std::byte>(c));
    return out;
}

}  // namespace

TEST_CASE("ProsFileStore: replacing a LONGER existing file leaves exactly the new bytes") {
    // BUG CAUGHT: mutation 12(a) — ProsBlockSink opening "ab" (append) instead of "wb": the file
    // would keep its old bytes and grow, and every record after the first would be refused by
    // length. Also any replace that writes without truncating.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsFileStore store{root.c_str()};
    const std::vector<std::byte> longer(300, std::byte{0x5A});
    REQUIRE(store.replace("shulib_drivetrain.bin", longer).failed == ReplaceStep::None);
    const std::vector<std::byte> record(shulib::calibration::kRecordBytes, std::byte{0x33});
    const StoreReplace w = store.replace("shulib_drivetrain.bin", record);
    CHECK(w.failed == ReplaceStep::None);
    const fs::path file = fs::path(root) / "shulib_drivetrain.bin";
    CHECK(fs::file_size(file) == shulib::calibration::kRecordBytes);
    CHECK(fileBytes(file) == record);
}

TEST_CASE("ProsFileStore: NO CARD -> replace reports NoCard, read is not opened, and NO file is "
          "created") {
    // BUG CAUGHT: a store that opens without asking the probe (on the host the open would succeed
    // and create the file — on the robot, a card-less write that "worked"), or one that reports a
    // refused open as NoCard with a card present.
    pros::shim::resetAll();  // the shim's adversarial default: no card
    const std::string root = scratchRoot();
    ProsFileStore store{root.c_str()};
    CHECK_FALSE(store.cardPresent());
    const std::vector<std::byte> record(16, std::byte{0x01});
    CHECK(store.replace("shulib_drivetrain.bin", record).failed == ReplaceStep::NoCard);
    std::array<std::byte, 32> into{};
    const StoreRead r = store.read("shulib_drivetrain.bin", into);
    CHECK_FALSE(r.opened);
    CHECK(r.count == 0);
    CHECK(fs::is_empty(fs::path(root)));

    // With a card, an open that fails (the root does not exist) is `Open`, not `NoCard`.
    pros::shim::usdState().installed = true;
    const std::string missingRoot = root + "no_such_dir/";
    ProsFileStore bad{missingRoot.c_str()};
    CHECK(bad.cardPresent());
    CHECK(bad.replace("x.bin", record).failed == ReplaceStep::Open);
    CHECK_FALSE(bad.read("x.bin", into).opened);
}

TEST_CASE("ProsFileStore: a write the device refuses reports Write, a flush it refuses reports Flush "
          "-- through the real adapter, against /dev/full") {
    // BUG CAUGHT: review mutant N08 (a failed flush never reported) and its twin for the write.
    // Both steps feed the words "(The card reported the write/flush of ... failing)", and Write is
    // one of the steps the save counts as "past its open". /dev/full accepts the open and refuses
    // every byte that reaches it (ENOSPC): 180 bytes wait in stdio's buffer until the flush
    // (Flush); 64 KiB overflow that buffer during the write (Write). Linux has /dev/full; a host
    // without it skips this case and says so.
    if (!fs::exists("/dev/full")) {
        MESSAGE("this host has no /dev/full: the Write/Flush steps are not exercised here");
        return;
    }
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    ProsFileStore device{"/dev/"};
    const std::vector<std::byte> record(shulib::calibration::kRecordBytes, std::byte{0x33});
    const StoreReplace small = device.replace("full", record);
    CHECK(small.failed == ReplaceStep::Flush);
    CHECK(std::string{small.path} == "/dev/full");
    const std::vector<std::byte> large(64 * 1024, std::byte{0x44});
    CHECK(device.replace("full", large).failed == ReplaceStep::Write);
}

TEST_CASE("ProsFileStore: a name that is not BARE throws from read AND from replace") {
    // BUG CAUGHT: a noexcept anywhere on the path (the precondition would become std::terminate),
    // or a store that joins its own prefix and so accepts "/usd/…" twice over.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsFileStore store{root.c_str()};
    std::array<std::byte, 8> into{};
    const std::vector<std::byte> b(4, std::byte{0});
    CHECK_THROWS_AS((void)store.read("/shulib_drivetrain.bin", into), PreconditionError);
    CHECK_THROWS_AS((void)store.replace("/shulib_drivetrain.bin", b), PreconditionError);
    CHECK_THROWS_AS((void)store.read("", into), PreconditionError);
    CHECK_THROWS_AS(ProsFileStore{nullptr}, PreconditionError);
    CHECK(fs::is_empty(fs::path(root)));
}

TEST_CASE("ProsFileStore: awkward bytes (NUL, LF, CR, 0x1A, 0xFF) survive a replace and a read") {
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsFileStore store{root.c_str()};
    const std::vector<std::byte> payload{std::byte{0x00}, std::byte{0x0A}, std::byte{0x0D},
                                         std::byte{0x0D}, std::byte{0x0A}, std::byte{0x1A},
                                         std::byte{0xFF}, std::byte{0x00}};
    REQUIRE(store.replace("rt.bin", payload).failed == ReplaceStep::None);
    std::array<std::byte, 64> into{};
    const StoreRead r = store.read("rt.bin", into);
    REQUIRE(r.opened);
    REQUIRE(r.count == payload.size());
    CHECK(std::vector<std::byte>(into.begin(), into.begin() + static_cast<std::ptrdiff_t>(r.count))
          == payload);
    // An EMPTY file opens and reads zero — not the same as no file.
    REQUIRE(store.replace("empty.bin", std::vector<std::byte>{}).failed == ReplaceStep::None);
    const StoreRead e = store.read("empty.bin", into);
    CHECK(e.opened);
    CHECK(e.count == 0);
}

TEST_CASE("ProsFileStore: each result's path is the ADAPTER's -- the injected root, or /usd/ from a "
          "default-constructed store") {
    // BUG CAUGHT: a store that builds its own path (a second spelling of the prefix, HA-122), or a
    // default constructor that passes a root of its own instead of letting the adapters' default
    // apply. The default-constructed case runs with the shim's card ABSENT, so nothing is touched.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsFileStore injected{root.c_str()};
    const std::vector<std::byte> b(4, std::byte{7});
    CHECK(std::string{injected.replace("shulib_drivetrain.bin", b).path}
          == root + "shulib_drivetrain.bin");
    std::array<std::byte, 8> into{};
    CHECK(std::string{injected.read("shulib_drivetrain.bin", into).path}
          == root + "shulib_drivetrain.bin");

    pros::shim::resetAll();  // no card: the default store touches nothing
    ProsFileStore robot;
    const StoreRead r = robot.read("shulib_drivetrain.bin", into);
    CHECK_FALSE(r.opened);
    CHECK(std::string{r.path} == "/usd/shulib_drivetrain.bin");
    const StoreReplace w = robot.replace("shulib_drivetrain.bak", b);
    CHECK(w.failed == ReplaceStep::NoCard);
    CHECK(std::string{w.path} == "/usd/shulib_drivetrain.bak");
}

TEST_CASE("ProsFileStore: the whole SAFE SAVE and LOAD run against real files -- both files hold the "
          "record, and a damaged record file falls back to the backup") {
    // The chunk's two library calls, end to end through the real adapters (host files).
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    ProsFileStore store{root.c_str()};

    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[2] = {-16, 17};
    const std::int8_t right[2] = {11, -12};
    for (int i = 0; i < 2; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 2;
    d.right.count = 2;
    d.left.inchesPerRadian = shulib::units::Length{1.25};
    d.right.inchesPerRadian = shulib::units::Length{1.37};
    d.trackWidth = shulib::units::Length{12.0};
    d.source = shulib::hal::DescriptionSource::RobotMeasured;

    const shulib::calibration::SaveReport rep = shulib::calibration::saveRecordSafely(store, d);
    CHECK(rep.outcome == shulib::calibration::SaveResult::Saved);
    CHECK(rep.backupAfter == shulib::calibration::FileHolds::New);
    const fs::path p = fs::path(root) / shulib::calibration::kRecordFileName;
    const fs::path k = fs::path(root) / shulib::calibration::kRecordBackupFileName;
    CHECK(fs::file_size(p) == shulib::calibration::kRecordBytes);
    CHECK(fileBytes(p) == fileBytes(k));
    CHECK(std::string{rep.primaryPath} == p.string());

    // Tear the record file on disk (as a laptop could); the load must take the backup.
    {
        std::ofstream out{p, std::ios::binary | std::ios::trunc};
        out << "torn";
    }
    const shulib::calibration::LoadedRecord rec = shulib::calibration::loadRecord(store);
    CHECK(rec.from == shulib::calibration::Which::Backup);
    CHECK(rec.primary.count == 4);
    CHECK_FALSE(rec.primary.decoded);
    CHECK(rec.description.trackWidth.value() == 12.0);
}
