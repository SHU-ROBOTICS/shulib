#pragma once
//
// ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk
// R1b): where the blackbox's binary blocks physically go. The device seam E1
// shipped the interface for (block_sink.hpp names R1 as this file's owner);
// diag/'s SdSink writes through it.
//
// BINDS:
//  * ::pros::usd::is_installed() — the no-card probe, at construction
//    (vendored misc.hpp:555-568; HA-122)
//  * std::fopen/std::fwrite/std::fflush on "/usd/<name>" — the PROS kernel
//    mounts the SD card at /usd/ for newlib file IO (HA-122)
//
// ── THE /usd/ PATH QUIRK (HA-122 — register + FAQ) ─────────────────────────────────
// Two conventions in ONE API, the same shape as R1a's 0-vs-1-indexed registry
// finding: usd_list_files() documents "DO NOT PREPEND YOUR PATHS WITH /usd/"
// (vendored misc.h:824-825) while fopen REQUIRES the /usd/ prefix. This
// adapter owns the prefix in exactly one place: the constructor takes a BARE
// file name (a leading '/' is a loud precondition, so the double-prefix
// mistake cannot compile out of sight), and joins it to `mountRoot` —
// "/usd/" on the robot, injectable so a host test can point it at a real
// temp directory and exercise THIS code path, not a copy of it.
//
// ── NO CARD AT BOOT (T5's ruling) ──────────────────────────────────────────────────
// Construction SUCCEEDS, write() returns false from the first call, and the
// fact is visible through isOpen() — checked once by the composition root /
// diagnostics layer, which owns saying it out loud (hal/ is below diag/). A
// missing SD card must not stop a robot from driving, and E1's drop-and-
// count design already handles a sink that refuses; a throw here would turn
// a missing card into a dead robot. Rejected: silently succeeding — the
// exact "nothing looks wrong" failure the blackbox exists to avoid.
//
// CONTRACT (block_sink.hpp): bytes VERBATIM, synchronous, MUST NOT throw —
// fwrite/fflush cannot throw, and both paths return bool. A short fwrite
// (card full, yanked, dying) returns false and leaves a prefix on the
// device, which is exactly what the format is designed to decode up to.
// flush() is fflush() — pushing newlib's buffer to the FatFS driver; there
// is no fsync in PROS's exposed surface, so "on the medium" is as strong as
// the platform allows (HA-122's weak half — bench: yank the card after a
// flushed write and count what survived).
//
// OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
// non-copyable/non-movable — unlike ProsCharSink, which borrows stdout.
//
// HA register: HA-122 (docs/hardware-assumptions.md).

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
#include "shulib/hal/block_sink.hpp"

namespace shulib::hal::pros {

class ProsBlockSink final : public IBlockSink {
public:
    /// Open `<mountRoot><fileName>` for binary writing (truncating — one
    /// blackbox file per boot). `fileName` must be BARE: no leading '/' (the
    /// adapter owns the /usd/ prefix — header note). `mountRoot` must end in
    /// '/'; it defaults to the robot truth and exists so host tests exercise
    /// the same join+open path. No card / failed open → a refusing sink, NOT
    /// a throw (T5): isOpen() false, every write()/flush() false.
    explicit ProsBlockSink(const char* fileName, const char* mountRoot = "/usd/") {
        SHULIB_PRECONDITION(fileName != nullptr && fileName[0] != '\0',
                            "ProsBlockSink: fileName is null/empty");
        SHULIB_PRECONDITION(fileName[0] != '/',
                            "ProsBlockSink: fileName must be BARE — the adapter owns the "
                            "/usd/ prefix (do not pre-prefix; HA-122)");
        SHULIB_PRECONDITION(mountRoot != nullptr && mountRoot[0] != '\0',
                            "ProsBlockSink: mountRoot is null/empty");
        SHULIB_PRECONDITION(std::string_view{mountRoot}.back() == '/',
                            "ProsBlockSink: mountRoot must end in '/'");
        path_ = std::string{mountRoot} + fileName;
        if (::pros::usd::is_installed() != 0) {  // the no-card probe (HA-122)
            file_ = std::fopen(path_.c_str(), "wb");
        }
    }

    ~ProsBlockSink() override {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    // Owns the FILE* — copying would double-close it (header note).
    ProsBlockSink(const ProsBlockSink&) = delete;
    ProsBlockSink& operator=(const ProsBlockSink&) = delete;
    ProsBlockSink(ProsBlockSink&&) = delete;
    ProsBlockSink& operator=(ProsBlockSink&&) = delete;

    /// Verbatim bytes; false unless EVERY byte was accepted (a short write
    /// leaves a prefix — the format decodes up to the cut). False always
    /// while refusing (no card / failed open).
    [[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override {
        if (file_ == nullptr) {
            return false;
        }
        if (bytes.empty()) {
            return true;  // nothing to accept; also keeps fwrite off a null data()
        }
        return std::fwrite(bytes.data(), 1, bytes.size(), file_) == bytes.size();
    }

    /// fflush to the FatFS driver (the platform's strongest "on the medium" —
    /// header note). False on device failure or while refusing.
    bool flush() noexcept override {
        if (file_ == nullptr) {
            return false;
        }
        return std::fflush(file_) == 0;
    }

    /// False = the sink is refusing (no card at boot, or the open failed).
    /// The composition root checks this ONCE and reports through the
    /// diagnostics layer — the T5 visibility rule.
    [[nodiscard]] bool isOpen() const noexcept { return file_ != nullptr; }

    /// The full path this sink writes (for the one-time diagnostics line).
    [[nodiscard]] const char* path() const noexcept { return path_.c_str(); }

private:
    std::string path_;
    std::FILE* file_ = nullptr;
};

}  // namespace shulib::hal::pros
