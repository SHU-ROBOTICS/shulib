#pragma once
//
// ProsBlockSource — IBlockSource over a PROS FILE* on the V5's SD card (chunk R3d
// Part 2): where the calibration record is read back from. The mirror image of
// ProsBlockSink, deliberately line for line, because the two must agree about the
// path or "shulib Drive" reads a file "shulib Calibrate" never wrote.
//
// BINDS:
//  * ::pros::usd::is_installed() — the no-card probe, at construction
//    (vendored misc.hpp:555-568; HA-122)
//  * std::fopen(…, "rb") / std::fread on "/usd/<name>" — the PROS kernel mounts the
//    SD card at /usd/ for newlib file IO (HA-122). "rb" and not "r": the record is
//    binary and contains 0x1A, 0x0A and 0x00; a text-mode read is a corrupt record on
//    any platform that translates, and the host test runs the SAME call.
//
// ── THE /usd/ PATH QUIRK (HA-122 — register + FAQ), owned here too ──────────────────
// usd_list_files() documents "DO NOT PREPEND YOUR PATHS WITH /usd/" while fopen REQUIRES
// the prefix. This adapter owns the prefix in exactly one place: the constructor takes a
// BARE file name (a leading '/' is a loud precondition) and joins it to `mountRoot` —
// "/usd/" on the robot, injectable so a host test can point it at a real temp directory
// and exercise THIS code path rather than a copy of it.
//
// ── NO CARD, OR NO FILE (T5's ruling, and its read-side half) ──────────────────────
// Construction SUCCEEDS in both cases; readAll() returns 0 and isOpen() is false. A missing
// card must not stop a robot from driving, and a MISSING RECORD is the normal state of a
// robot that has never been calibrated — neither is an error, and a throw here would turn
// either into a dead robot at boot. The composition root asks isOpen() once and says which
// it was: "no SD card" and "no calibration on this card" send an operator to two different
// places, and the seam's contract (block_source.hpp) exists to keep them apart.
//
// A SHORT read is NOT an error here: the count comes back and the FORMAT refuses by name
// (calibration_record.hpp's length check). An adapter that turned a truncated file into a
// bool would have thrown away the one word the operator needs.
//
// OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
// non-copyable/non-movable — the sink's reasoning, unchanged: a second handle would fclose
// the same file twice.
//
// HA register: HA-122 (docs/hardware-assumptions.md), plus HA-139 for the "rb" belief.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include <cstddef>
#include <cstdio>
#include <span>
#include <string>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/block_source.hpp"

namespace shulib::hal::pros {

/// IBlockSource over a PROS FILE* on the V5's SD card — where the calibration record is read
/// back at boot. It OWNS the file: opened for binary reading at construction and closed at
/// destruction. A MISSING CARD OR A MISSING FILE IS NOT AN ERROR — construction still
/// succeeds and the source simply refuses (isOpen() false, readAll() 0), because a robot with
/// no calibration must still be able to boot and say so. It also owns the /usd/ prefix: pass
/// a BARE file name.
class ProsBlockSource final : public IBlockSource {
public:
    /// Open `<mountRoot><fileName>` for binary READING. `fileName` must be BARE: no leading
    /// '/' (the adapter owns the /usd/ prefix — header note). `mountRoot` must end in '/'; it
    /// defaults to the robot truth and exists so host tests exercise the same join+open path.
    /// No card, or a file that does not exist → a refusing source, NOT a throw (T5):
    /// isOpen() false, readAll() 0.
    explicit ProsBlockSource(const char* fileName, const char* mountRoot = "/usd/") {
        SHULIB_PRECONDITION(fileName != nullptr && fileName[0] != '\0',
                            "ProsBlockSource: fileName is null/empty");
        SHULIB_PRECONDITION(fileName[0] != '/',
                            "ProsBlockSource: fileName must be BARE — the adapter owns the "
                            "/usd/ prefix (do not pre-prefix; HA-122)");
        SHULIB_PRECONDITION(mountRoot != nullptr && mountRoot[0] != '\0',
                            "ProsBlockSource: mountRoot is null/empty");
        SHULIB_PRECONDITION(std::string_view{mountRoot}.back() == '/',
                            "ProsBlockSource: mountRoot must end in '/'");
        path_ = std::string{mountRoot} + fileName;
        if (::pros::usd::is_installed() != 0) {  // the no-card probe (HA-122)
            file_ = std::fopen(path_.c_str(), "rb");
        }
    }

    /// fclose the file. Nothing else holds this FILE*. No-op on a refusing source. Unlike the
    /// SINK's destructor there is no unreportable failure here: closing a file opened for
    /// reading flushes nothing and loses nothing.
    ~ProsBlockSource() override {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    /// Non-copyable and non-movable: this adapter OWNS the FILE*, and a second handle to it
    /// would fclose the same file twice (ProsBlockSink's reasoning, unchanged).
    ProsBlockSource(const ProsBlockSource&) = delete;
    ProsBlockSource& operator=(const ProsBlockSource&) = delete;
    ProsBlockSource(ProsBlockSource&&) = delete;
    ProsBlockSource& operator=(ProsBlockSource&&) = delete;

    /// Read up to `into.size()` bytes from the START of the file and return the count. Seeks
    /// to 0 first, so the seam's "reads from the start" contract holds even if somebody calls
    /// it twice. 0 while refusing, 0 on an empty file, and a short count on a truncated one —
    /// which the format refuses by name, not this adapter.
    [[nodiscard]] std::size_t readAll(std::span<std::byte> into) noexcept override {
        if (file_ == nullptr || into.empty()) {
            return 0U;
        }
        if (std::fseek(file_, 0, SEEK_SET) != 0) {
            return 0U;
        }
        return std::fread(into.data(), 1, into.size(), file_);
    }

    /// False = the source is refusing (no card at boot, or the file does not exist / failed to
    /// open). The composition root checks this ONCE and reports — the T5 visibility rule.
    [[nodiscard]] bool isOpen() const noexcept override { return file_ != nullptr; }

    /// The full path this source reads (for the one-time diagnostics line).
    [[nodiscard]] const char* path() const noexcept { return path_.c_str(); }

private:
    std::string path_;
    std::FILE* file_ = nullptr;
};

}  // namespace shulib::hal::pros
